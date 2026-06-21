#!/usr/bin/env python3
"""
Drone Controller Node.

Provides action servers for:
  - /drone/spray_zone  (SprayZone action): Executes lawnmower spray pattern
  - /drone/return_to_rover (ReturnToBase action): Precision landing via ArUco

Controls drone via MAVROS2:
  - Arming, mode setting, offboard position setpoints
  - PX4 NED <-> ROS2 ENU frame handled by MAVROS2 automatically
"""

import asyncio
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState

try:
    from mavros_msgs.msg import State
    from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
    MAVROS_AVAILABLE = True
except ImportError:
    MAVROS_AVAILABLE = False

from agri_interfaces.msg import DroneStatus
from agri_interfaces.action import SprayZone, ReturnToBase


# Battery level below which drone aborts mission and returns
LOW_BATTERY_THRESHOLD = 0.25
# Tolerance for reaching a waypoint (meters)
WAYPOINT_TOLERANCE = 0.5
# ArUco detection altitude - descend to this height to search for marker
ARUCO_SEARCH_ALTITUDE = 3.0
# Altitude for precision landing approach
ARUCO_LOCK_ALTITUDE = 0.5
# Minimum cycles of setpoint publishing before engaging offboard mode
OFFBOARD_PREARM_CYCLES = 100


class DroneControllerNode(Node):
    def __init__(self):
        super().__init__('drone_controller')

        self.declare_parameter('takeoff_altitude', 5.0)
        self.declare_parameter('spray_altitude', 3.0)
        self.declare_parameter('max_velocity', 3.0)
        self.declare_parameter('aruco_kp_xy', 0.4)
        self.declare_parameter('aruco_kd_xy', 0.05)
        self.declare_parameter('descent_rate', 0.3)
        self.declare_parameter('use_sim_time', True)

        self.takeoff_alt = self.get_parameter('takeoff_altitude').value
        self.spray_alt = self.get_parameter('spray_altitude').value
        self.max_vel = self.get_parameter('max_velocity').value
        self.kp_xy = self.get_parameter('aruco_kp_xy').value
        self.kd_xy = self.get_parameter('aruco_kd_xy').value
        self.descent_rate = self.get_parameter('descent_rate').value

        # Drone state
        self.current_pose = PoseStamped()
        self.current_state = None
        self.battery_percent = 1.0
        self.aruco_pose = None
        self.aruco_detected = False
        self.last_aruco_error = np.zeros(2)
        self.is_airborne = False

        cb_group = ReentrantCallbackGroup()

        # MAVROS2 subscribers
        if MAVROS_AVAILABLE:
            self.state_sub = self.create_subscription(
                State, '/mavros/state', self.state_callback, 10,
                callback_group=cb_group)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_callback, 20,
            callback_group=cb_group)
        self.battery_sub = self.create_subscription(
            BatteryState, '/mavros/battery', self.battery_callback, 10,
            callback_group=cb_group)

        # ArUco subscribers
        self.aruco_pose_sub = self.create_subscription(
            PoseStamped, '/aruco/pose', self.aruco_pose_callback, 20,
            callback_group=cb_group)
        self.aruco_detected_sub = self.create_subscription(
            Bool, '/aruco/detection_status', self.aruco_detected_callback, 10,
            callback_group=cb_group)

        # Publishers
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 20)
        self.vel_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 20)
        self.status_pub = self.create_publisher(DroneStatus, '/drone/status', 10)

        # MAVROS2 service clients
        if MAVROS_AVAILABLE:
            self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming',
                                                     callback_group=cb_group)
            self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode',
                                                       callback_group=cb_group)
            self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land',
                                                   callback_group=cb_group)

        # Action servers
        self.spray_action_server = ActionServer(
            self,
            SprayZone,
            '/drone/spray_zone',
            execute_callback=self.spray_zone_execute,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda cancel: CancelResponse.ACCEPT,
            callback_group=cb_group,
        )

        self.return_action_server = ActionServer(
            self,
            ReturnToBase,
            '/drone/return_to_rover',
            execute_callback=self.return_to_rover_execute,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda cancel: CancelResponse.ACCEPT,
            callback_group=cb_group,
        )

        # Status timer
        self.status_timer = self.create_timer(0.5, self.publish_status, callback_group=cb_group)

        self.get_logger().info('Drone Controller Node started.')

    # ── Callbacks ────────────────────────────────────────────────────────────

    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg
        self.is_airborne = msg.pose.position.z > 0.3

    def battery_callback(self, msg: BatteryState):
        self.battery_percent = msg.percentage if msg.percentage >= 0.0 else 1.0

    def aruco_pose_callback(self, msg: PoseStamped):
        self.aruco_pose = msg

    def aruco_detected_callback(self, msg: Bool):
        self.aruco_detected = msg.data

    def publish_status(self):
        msg = DroneStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.current_state:
            msg.flight_mode = self.current_state.mode
            msg.is_armed = self.current_state.armed
            msg.is_connected = self.current_state.connected
        msg.altitude_m = self.current_pose.pose.position.z
        msg.battery_percent = self.battery_percent
        msg.position = Point(
            x=self.current_pose.pose.position.x,
            y=self.current_pose.pose.position.y,
            z=self.current_pose.pose.position.z,
        )
        self.status_pub.publish(msg)

    # ── MAVROS2 Helpers ───────────────────────────────────────────────────────

    def send_setpoint(self, x: float, y: float, z: float, yaw: float = 0.0):
        """Send position setpoint in local ENU frame."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        # Convert yaw to quaternion (rotation around Z)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.setpoint_pub.publish(msg)

    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0):
        """Send velocity setpoint in body frame."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        msg.twist.angular.z = yaw_rate
        self.vel_pub.publish(msg)

    async def arm_and_set_offboard(self) -> bool:
        """Pre-publish setpoints, then arm and set OFFBOARD mode."""
        if not MAVROS_AVAILABLE:
            self.get_logger().warn('MAVROS not available - simulating arm/offboard')
            return True

        # Pre-publish setpoints to satisfy PX4 offboard requirement
        self.get_logger().info('Pre-publishing setpoints for OFFBOARD mode...')
        rate = self.create_rate(20)
        home_x = self.current_pose.pose.position.x
        home_y = self.current_pose.pose.position.y
        for _ in range(OFFBOARD_PREARM_CYCLES):
            self.send_setpoint(home_x, home_y, self.takeoff_alt)
            await asyncio.sleep(0.05)

        # Set OFFBOARD mode
        self.get_logger().info('Setting OFFBOARD mode...')
        set_mode_req = SetMode.Request()
        set_mode_req.custom_mode = 'OFFBOARD'
        future = self.set_mode_client.call_async(set_mode_req)
        await future
        if not future.result().mode_sent:
            self.get_logger().error('Failed to set OFFBOARD mode')
            return False

        # Arm
        self.get_logger().info('Arming drone...')
        arm_req = CommandBool.Request()
        arm_req.value = True
        future = self.arming_client.call_async(arm_req)
        await future
        if not future.result().success:
            self.get_logger().error('Failed to arm drone')
            return False

        self.get_logger().info('Drone armed and in OFFBOARD mode.')
        return True

    async def disarm(self):
        if not MAVROS_AVAILABLE:
            return
        req = CommandBool.Request()
        req.value = False
        await self.arming_client.call_async(req)

    def distance_to(self, x: float, y: float, z: float) -> float:
        dx = self.current_pose.pose.position.x - x
        dy = self.current_pose.pose.position.y - y
        dz = self.current_pose.pose.position.z - z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    async def fly_to(self, x: float, y: float, z: float,
                     tolerance: float = WAYPOINT_TOLERANCE,
                     timeout: float = 30.0) -> bool:
        """Fly to position and wait until reached or timeout."""
        start = time.time()
        while rclpy.ok():
            self.send_setpoint(x, y, z)
            if self.distance_to(x, y, z) < tolerance:
                return True
            if time.time() - start > timeout:
                self.get_logger().warn(f'Timeout flying to ({x:.1f},{y:.1f},{z:.1f})')
                return False
            await asyncio.sleep(0.05)
        return False

    # ── Spray Zone Action ─────────────────────────────────────────────────────

    async def spray_zone_execute(self, goal_handle):
        """Execute lawnmower spray pattern over zone."""
        goal = goal_handle.request
        self.get_logger().info(f'Starting spray mission for zone {goal.zone_id}')
        feedback = SprayZone.Feedback()
        result = SprayZone.Result()

        # Arm and enter offboard mode
        if not await self.arm_and_set_offboard():
            result.success = False
            result.actual_coverage = 0.0
            goal_handle.abort()
            return result

        # Takeoff
        home_x = self.current_pose.pose.position.x
        home_y = self.current_pose.pose.position.y
        self.get_logger().info(f'Taking off to {self.takeoff_alt}m...')
        if not await self.fly_to(home_x, home_y, self.takeoff_alt, timeout=20.0):
            result.success = False
            goal_handle.abort()
            return result
        self.get_logger().info('Takeoff complete.')

        # Execute waypoints from goal
        waypoints = goal.waypoints.poses
        total = len(waypoints)
        if total == 0:
            self.get_logger().error('No waypoints received for spray pattern')
            result.success = False
            goal_handle.abort()
            return result

        for idx, wp in enumerate(waypoints):
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Spray mission cancelled')
                goal_handle.canceled()
                result.success = False
                return result

            if self.battery_percent < LOW_BATTERY_THRESHOLD:
                self.get_logger().warn('Low battery - aborting spray')
                break

            target_alt = goal.altitude if goal.altitude > 0 else self.spray_alt
            success = await self.fly_to(
                wp.position.x, wp.position.y, target_alt,
                tolerance=WAYPOINT_TOLERANCE, timeout=60.0
            )
            if not success:
                self.get_logger().warn(f'Could not reach waypoint {idx+1}/{total}')

            # Publish feedback every other waypoint
            strips_done = (idx + 1) // 2
            total_strips = total // 2
            feedback.strips_completed = strips_done
            feedback.total_strips = max(total_strips, 1)
            feedback.coverage_percent = float(idx + 1) / total * 100.0
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(
                f'Spray progress: {idx+1}/{total} waypoints, '
                f'{feedback.coverage_percent:.0f}% coverage'
            )

        result.success = True
        result.actual_coverage = feedback.coverage_percent
        goal_handle.succeed()
        self.get_logger().info(f'Spray complete. Coverage: {result.actual_coverage:.0f}%')
        return result

    # ── Return to Rover (ArUco Precision Landing) Action ─────────────────────

    async def return_to_rover_execute(self, goal_handle):
        """
        Return drone to rover cart using ArUco precision landing.

        Phases:
        1. Fly to rover position at safe altitude
        2. Descend to ArUco search altitude
        3. Visual servo using ArUco offset
        4. Precision descent and land
        """
        goal = goal_handle.request
        feedback = ReturnToBase.Feedback()
        result = ReturnToBase.Result()
        aruco_used = False

        rover_x = goal.target_pose.pose.position.x
        rover_y = goal.target_pose.pose.position.y

        self.get_logger().info(
            f'Returning to rover at ({rover_x:.1f}, {rover_y:.1f})...')

        # Phase 1: Fly to above rover at safe altitude
        feedback.landing_phase = 'TRANSIT'
        goal_handle.publish_feedback(feedback)
        safe_alt = max(self.takeoff_alt, self.current_pose.pose.position.z)
        await self.fly_to(rover_x, rover_y, safe_alt, tolerance=1.0, timeout=60.0)
        self.get_logger().info('Phase 1: Over rover. Descending to ArUco search altitude...')

        # Phase 2: Descend to ArUco acquisition altitude
        feedback.landing_phase = 'ARUCO_SEARCH'
        goal_handle.publish_feedback(feedback)
        await self.fly_to(rover_x, rover_y, ARUCO_SEARCH_ALTITUDE, tolerance=0.3, timeout=20.0)

        # Phase 3: Wait for ArUco detection (up to 10 seconds)
        self.get_logger().info('Phase 2: Searching for ArUco marker...')
        search_start = time.time()
        while rclpy.ok() and not self.aruco_detected:
            if time.time() - search_start > 10.0:
                self.get_logger().warn('ArUco not found - falling back to direct landing')
                break
            self.send_setpoint(rover_x, rover_y, ARUCO_SEARCH_ALTITUDE)
            await asyncio.sleep(0.05)

        if self.aruco_detected:
            aruco_used = True
            self.get_logger().info('Phase 3: ArUco acquired. Starting visual servo...')
            # Phase 3: Visual servo (PD controller on ArUco offset)
            prev_error = np.zeros(2)
            feedback.landing_phase = 'ARUCO_TRACKING'
            goal_handle.publish_feedback(feedback)

            landing_timeout = time.time() + 30.0
            while rclpy.ok() and self.current_pose.pose.position.z > ARUCO_LOCK_ALTITUDE:
                if time.time() > landing_timeout:
                    self.get_logger().warn('ArUco landing timeout')
                    break

                if self.aruco_pose is not None and self.aruco_detected:
                    # Camera optical frame: x=right, y=down, z=forward (distance)
                    # We want to minimize x and y offset
                    error_x = self.aruco_pose.pose.position.x  # right offset
                    error_y = self.aruco_pose.pose.position.y  # down offset (depth from camera)
                    # In ENU body frame: drone_x = -camera_y, drone_y = -camera_x
                    # (camera pointing down, x axis points drone-right)
                    current_error = np.array([error_x, error_y])
                    d_error = current_error - prev_error

                    vx = -(self.kp_xy * error_y + self.kd_xy * d_error[1])
                    vy = -(self.kp_xy * error_x + self.kd_xy * d_error[0])
                    vx = float(np.clip(vx, -1.0, 1.0))
                    vy = float(np.clip(vy, -1.0, 1.0))

                    # Descend when error is small
                    xy_error_norm = math.sqrt(error_x**2 + error_y**2)
                    vz = -self.descent_rate if xy_error_norm < 0.15 else 0.0

                    self.send_velocity(vx, vy, vz)
                    prev_error = current_error

                    feedback.landing_phase = (
                        f'DESCENDING_{self.current_pose.pose.position.z:.1f}m'
                    )
                    feedback.distance_remaining = self.current_pose.pose.position.z
                    goal_handle.publish_feedback(feedback)
                else:
                    # Lost ArUco - hold position
                    self.send_setpoint(
                        self.current_pose.pose.position.x,
                        self.current_pose.pose.position.y,
                        self.current_pose.pose.position.z,
                    )

                await asyncio.sleep(0.05)

        # Final landing
        self.get_logger().info('Phase 4: Final landing...')
        feedback.landing_phase = 'LANDING'
        goal_handle.publish_feedback(feedback)

        if MAVROS_AVAILABLE:
            land_req = CommandTOL.Request()
            land_req.altitude = 0.0
            await self.land_client.call_async(land_req)
        else:
            # Simulate landing
            for alt in [ARUCO_LOCK_ALTITUDE, 0.0]:
                self.send_setpoint(
                    self.current_pose.pose.position.x,
                    self.current_pose.pose.position.y,
                    alt
                )
                await asyncio.sleep(1.0)

        # Wait for landing
        await asyncio.sleep(3.0)

        # Disarm
        await self.disarm()

        result.success = True
        result.aruco_landing_used = aruco_used
        goal_handle.succeed()
        self.get_logger().info(
            f'Drone landed. ArUco used: {aruco_used}')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DroneControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
