#!/usr/bin/env python3
"""
Rover Controller Node.

Wraps Nav2's NavigateToPose action and provides:
  - /rover/navigate_to_zone (ExecuteMission action): Navigate to a named zone
  - /rover/return_to_base (ReturnToBase action): Return rover to shed

Publishes current rover pose and zone status.
Zone coordinates are loaded from mission_config.yaml.
"""

import math
import yaml
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

try:
    from nav2_msgs.action import NavigateToPose
    NAV2_AVAILABLE = True
except ImportError:
    NAV2_AVAILABLE = False

from agri_interfaces.msg import RoverStatus
from agri_interfaces.action import ExecuteMission, ReturnToBase


# Default zone positions in local ENU frame (meters from map origin/shed)
DEFAULT_ZONES = {
    'A': {'x': 50.0, 'y': 80.0, 'yaw': 0.0, 'description': 'North Field'},
    'B': {'x': 120.0, 'y': 50.0, 'yaw': 0.0, 'description': 'East Field'},
    'C': {'x': 50.0, 'y': -60.0, 'yaw': 0.0, 'description': 'South Field'},
    'D': {'x': -70.0, 'y': 40.0, 'yaw': 0.0, 'description': 'West Field'},
    'BASE': {'x': 0.0, 'y': 0.0, 'yaw': 3.14159, 'description': 'Shed Base'},
}


class RoverControllerNode(Node):
    def __init__(self):
        super().__init__('rover_controller')

        self.declare_parameter('zones_config_file', '')
        self.declare_parameter('navigation_timeout_s', 120.0)
        self.declare_parameter('goal_tolerance_m', 0.5)

        config_file = self.get_parameter('zones_config_file').value
        self.nav_timeout = self.get_parameter('navigation_timeout_s').value
        self.goal_tolerance = self.get_parameter('goal_tolerance_m').value

        # Load zone coordinates
        self.zones = DEFAULT_ZONES.copy()
        if config_file and os.path.exists(config_file):
            self._load_zones_config(config_file)

        # Robot state
        self.current_pose = PoseStamped()
        self.current_zone = 'UNKNOWN'

        cb_group = ReentrantCallbackGroup()

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/rover/odom', self.odom_callback, 10,
            callback_group=cb_group)

        # Publishers
        self.status_pub = self.create_publisher(RoverStatus, '/rover/status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/rover/cmd_vel', 10)

        # Nav2 action client
        if NAV2_AVAILABLE:
            self.nav2_client = ActionClient(
                self, NavigateToPose, '/navigate_to_pose',
                callback_group=cb_group)
            self.get_logger().info('Waiting for Nav2 NavigateToPose action server...')
            self.nav2_client.wait_for_server(timeout_sec=10.0)
            self.get_logger().info('Nav2 connected.')
        else:
            self.nav2_client = None
            self.get_logger().warn('nav2_msgs not available - using simulated navigation')

        # Action servers
        self.navigate_action_server = ActionServer(
            self,
            ExecuteMission,
            '/rover/navigate_to_zone',
            execute_callback=self.navigate_to_zone_execute,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda cancel: CancelResponse.ACCEPT,
            callback_group=cb_group,
        )

        self.return_action_server = ActionServer(
            self,
            ReturnToBase,
            '/rover/return_to_base',
            execute_callback=self.return_to_base_execute,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda cancel: CancelResponse.ACCEPT,
            callback_group=cb_group,
        )

        # Status timer
        self.status_timer = self.create_timer(
            0.5, self.publish_status, callback_group=cb_group)

        self.get_logger().info(
            f'Rover Controller started. Known zones: {list(self.zones.keys())}')

    def _load_zones_config(self, path: str):
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            if 'field_zones' in data:
                for zone_id, info in data['field_zones'].items():
                    lf = info.get('local_frame', {})
                    self.zones[zone_id] = {
                        'x': lf.get('x', 0.0),
                        'y': lf.get('y', 0.0),
                        'yaw': lf.get('yaw', 0.0),
                        'description': info.get('name', zone_id),
                    }
            if 'base' in data:
                lf = data['base'].get('local_frame', {})
                self.zones['BASE'] = {
                    'x': lf.get('x', 0.0),
                    'y': lf.get('y', 0.0),
                    'yaw': lf.get('yaw', math.pi),
                    'description': 'Base/Shed',
                }
            self.get_logger().info(f'Loaded zone config from {path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load zones config: {e}')

    def odom_callback(self, msg: Odometry):
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose
        # Update zone estimate
        self.current_zone = self._nearest_zone()

    def _nearest_zone(self) -> str:
        min_dist = float('inf')
        nearest = 'TRANSIT'
        cx = self.current_pose.pose.position.x
        cy = self.current_pose.pose.position.y
        for zone_id, info in self.zones.items():
            d = math.sqrt((cx - info['x'])**2 + (cy - info['y'])**2)
            if d < min_dist:
                min_dist = d
                nearest = zone_id
        return nearest if min_dist < 5.0 else 'TRANSIT'

    def publish_status(self):
        msg = RoverStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.current_pose = self.current_pose.pose
        msg.current_zone = self.current_zone
        self.status_pub.publish(msg)

    def _build_nav2_goal(self, x: float, y: float, yaw: float) -> 'NavigateToPose.Goal':
        if not NAV2_AVAILABLE:
            return None
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    async def _navigate_to_xy(self, x: float, y: float, yaw: float,
                               goal_handle, feedback_msg) -> bool:
        """Navigate to position using Nav2 or simulated movement."""
        if NAV2_AVAILABLE and self.nav2_client:
            nav_goal = self._build_nav2_goal(x, y, yaw)
            send_goal_future = self.nav2_client.send_goal_async(
                nav_goal,
                feedback_callback=lambda fb: None
            )
            await send_goal_future
            nav_handle = send_goal_future.result()
            if not nav_handle.accepted:
                self.get_logger().error('Nav2 rejected goal')
                return False

            result_future = nav_handle.get_result_async()
            # Poll while waiting for Nav2 to finish
            import asyncio
            while not result_future.done():
                if goal_handle.is_cancel_requested:
                    await nav_handle.cancel_goal_async()
                    return False
                dx = x - self.current_pose.pose.position.x
                dy = y - self.current_pose.pose.position.y
                dist = math.sqrt(dx*dx + dy*dy)
                feedback_msg.progress = max(0.0, 1.0 - dist / max(dist, 1.0))
                goal_handle.publish_feedback(feedback_msg)
                await asyncio.sleep(0.5)

            nav_result = result_future.result()
            return nav_result.result.error_code == 0
        else:
            # Simulated navigation: publish cmd_vel toward goal
            return await self._simulated_navigate(x, y, goal_handle, feedback_msg)

    async def _simulated_navigate(self, x: float, y: float,
                                   goal_handle, feedback_msg) -> bool:
        """Publish cmd_vel to drive toward target (simulation fallback)."""
        import asyncio
        self.get_logger().info(f'[SIM] Navigating to ({x:.1f}, {y:.1f})')
        timeout = self.nav_timeout
        elapsed = 0.0
        dt = 0.1
        speed = 0.5  # m/s

        while rclpy.ok() and elapsed < timeout:
            if goal_handle.is_cancel_requested:
                self._stop_rover()
                return False

            cx = self.current_pose.pose.position.x
            cy = self.current_pose.pose.position.y
            dx = x - cx
            dy = y - cy
            dist = math.sqrt(dx*dx + dy*dy)

            if dist < self.goal_tolerance:
                self._stop_rover()
                return True

            # Simple proportional heading control
            heading = math.atan2(dy, dx)
            twist = Twist()
            twist.linear.x = min(speed, dist * 0.5)
            # Angular velocity to face goal
            current_yaw = self._get_yaw()
            heading_error = heading - current_yaw
            # Normalize to [-pi, pi]
            while heading_error > math.pi:
                heading_error -= 2 * math.pi
            while heading_error < -math.pi:
                heading_error += 2 * math.pi
            twist.angular.z = float(min(max(heading_error * 1.0, -1.0), 1.0))

            self.cmd_vel_pub.publish(twist)
            feedback_msg.progress = max(0.0, 1.0 - dist / 100.0)
            goal_handle.publish_feedback(feedback_msg)
            await asyncio.sleep(dt)
            elapsed += dt

        self._stop_rover()
        return False

    def _stop_rover(self):
        self.cmd_vel_pub.publish(Twist())

    def _get_yaw(self) -> float:
        q = self.current_pose.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    async def navigate_to_zone_execute(self, goal_handle):
        """Navigate rover to named field zone."""
        zone_id = goal_handle.request.command_text.strip().upper()
        result = ExecuteMission.Result()
        feedback = ExecuteMission.Feedback()

        if zone_id not in self.zones:
            self.get_logger().error(f'Unknown zone: {zone_id}')
            result.success = False
            result.error_message = f'Unknown zone: {zone_id}'
            goal_handle.abort()
            return result

        zone = self.zones[zone_id]
        self.get_logger().info(
            f'Navigating to zone {zone_id}: ({zone["x"]:.1f}, {zone["y"]:.1f})')

        feedback.current_state = f'NAVIGATING_TO_{zone_id}'
        feedback.status_message = f'Heading to {zone["description"]}'
        goal_handle.publish_feedback(feedback)

        success = await self._navigate_to_xy(
            zone['x'], zone['y'], zone['yaw'], goal_handle, feedback)

        if success:
            self.current_zone = zone_id
            result.success = True
            result.final_state = f'ARRIVED_{zone_id}'
            goal_handle.succeed()
            self.get_logger().info(f'Arrived at zone {zone_id}')
        else:
            result.success = False
            result.error_message = f'Navigation to zone {zone_id} failed'
            goal_handle.abort()

        return result

    async def return_to_base_execute(self, goal_handle):
        """Navigate rover back to shed/base."""
        result = ReturnToBase.Result()
        feedback = ReturnToBase.Feedback()
        feedback.landing_phase = 'RETURNING_TO_BASE'
        goal_handle.publish_feedback(feedback)

        base = self.zones.get('BASE', {'x': 0.0, 'y': 0.0, 'yaw': math.pi})
        self.get_logger().info('Returning rover to base...')

        success = await self._navigate_to_xy(
            base['x'], base['y'], base['yaw'], goal_handle, feedback)

        result.success = success
        result.aruco_landing_used = False
        if success:
            self.current_zone = 'BASE'
            goal_handle.succeed()
            self.get_logger().info('Rover returned to base.')
        else:
            result.success = False
            goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)
    node = RoverControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
