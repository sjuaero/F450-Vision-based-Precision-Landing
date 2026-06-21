#!/usr/bin/env python3
"""
Mission Manager Node - Central orchestrator for the agricultural drone-rover system.

State Machine:
  IDLE
   └─→ VALIDATING_COMMAND     (LLM parses user command)
        └─→ TRANSITING_TO_ZONE (rover drives to field)
             └─→ DEPLOYING_DRONE  (drone arms + takes off)
                  └─→ SPRAYING    (drone executes lawnmower pattern)
                       └─→ DRONE_RETURNING (drone lands on rover cart via ArUco)
                            └─→ RETURNING_TO_BASE (rover drives back to shed)
                                 └─→ IDLE

Any state can transition to EMERGENCY on error.
EMERGENCY requires explicit /mission/reset to return to IDLE.

Subscribes:
  /mission/command_raw    (std_msgs/String) - raw user command
  /mission/reset          (std_msgs/Bool)   - emergency reset
  /drone/status           (DroneStatus)
  /rover/status           (RoverStatus)
  /mission/command_parsed (ZoneInfo)        - from LLM node

Publishes:
  /mission/status         (MissionStatus)

Action Clients:
  /rover/navigate_to_zone (ExecuteMission)
  /rover/return_to_base   (ReturnToBase)
  /drone/spray_zone       (SprayZone)
  /drone/return_to_rover  (ReturnToBase)

Services:
  /llm/parse_command      (ParseMissionCommand)
"""

import asyncio
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped

from agri_interfaces.msg import MissionStatus, ZoneInfo, DroneStatus, RoverStatus
from agri_interfaces.action import ExecuteMission, SprayZone, ReturnToBase
from agri_interfaces.srv import ParseMissionCommand

from agri_drone_controller.spray_pattern_executor import SprayPatternExecutor


class MissionState(Enum):
    IDLE = 'IDLE'
    VALIDATING_COMMAND = 'VALIDATING_COMMAND'
    TRANSITING_TO_ZONE = 'TRANSITING_TO_ZONE'
    DEPLOYING_DRONE = 'DEPLOYING_DRONE'
    SPRAYING = 'SPRAYING'
    DRONE_RETURNING = 'DRONE_RETURNING'
    RETURNING_TO_BASE = 'RETURNING_TO_BASE'
    EMERGENCY = 'EMERGENCY'


# Valid state transitions
VALID_TRANSITIONS = {
    MissionState.IDLE: [MissionState.VALIDATING_COMMAND],
    MissionState.VALIDATING_COMMAND: [
        MissionState.TRANSITING_TO_ZONE, MissionState.IDLE, MissionState.EMERGENCY],
    MissionState.TRANSITING_TO_ZONE: [
        MissionState.DEPLOYING_DRONE, MissionState.IDLE, MissionState.EMERGENCY],
    MissionState.DEPLOYING_DRONE: [MissionState.SPRAYING, MissionState.EMERGENCY],
    MissionState.SPRAYING: [MissionState.DRONE_RETURNING, MissionState.EMERGENCY],
    MissionState.DRONE_RETURNING: [MissionState.RETURNING_TO_BASE, MissionState.EMERGENCY],
    MissionState.RETURNING_TO_BASE: [MissionState.IDLE, MissionState.EMERGENCY],
    MissionState.EMERGENCY: [MissionState.IDLE],
}

# Default zone configs for spray pattern generation
ZONES_CONFIG = {
    'A': {'center_x': 50.0, 'center_y': 80.0, 'width': 50.0, 'length': 80.0,
          'spray_altitude': 3.0, 'spray_width': 4.0, 'overlap': 0.20},
    'B': {'center_x': 120.0, 'center_y': 50.0, 'width': 60.0, 'length': 70.0,
          'spray_altitude': 3.0, 'spray_width': 4.0, 'overlap': 0.20},
    'C': {'center_x': 50.0, 'center_y': -60.0, 'width': 55.0, 'length': 75.0,
          'spray_altitude': 3.0, 'spray_width': 4.0, 'overlap': 0.20},
    'D': {'center_x': -70.0, 'center_y': 40.0, 'width': 45.0, 'length': 80.0,
          'spray_altitude': 3.0, 'spray_width': 4.0, 'overlap': 0.20},
}


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager')

        self.declare_parameter('min_confidence_threshold', 0.6)
        self.declare_parameter('action_timeout_s', 300.0)
        self.min_confidence = self.get_parameter('min_confidence_threshold').value
        self.action_timeout = self.get_parameter('action_timeout_s').value

        # State machine
        self.state = MissionState.IDLE
        self.active_zone = ''
        self.last_error = ''
        self.mission_progress = 0.0

        # Latest sensor data
        self.rover_pose = PoseStamped()
        self.drone_status = DroneStatus()
        self.rover_status = RoverStatus()
        self.pending_zone_info: ZoneInfo = None

        self.spray_executor = SprayPatternExecutor()

        cb_group = ReentrantCallbackGroup()

        # Subscribers
        self.cmd_sub = self.create_subscription(
            String, '/mission/command_raw', self.command_raw_callback, 10,
            callback_group=cb_group)
        self.reset_sub = self.create_subscription(
            Bool, '/mission/reset', self.reset_callback, 10,
            callback_group=cb_group)
        self.drone_status_sub = self.create_subscription(
            DroneStatus, '/drone/status', self.drone_status_callback, 10,
            callback_group=cb_group)
        self.rover_status_sub = self.create_subscription(
            RoverStatus, '/rover/status', self.rover_status_callback, 10,
            callback_group=cb_group)

        # Publishers
        self.status_pub = self.create_publisher(MissionStatus, '/mission/status', 10)

        # LLM service client
        self.llm_client = self.create_client(
            ParseMissionCommand, '/llm/parse_command',
            callback_group=cb_group)

        # Action clients
        self.rover_nav_client = ActionClient(
            self, ExecuteMission, '/rover/navigate_to_zone',
            callback_group=cb_group)
        self.rover_return_client = ActionClient(
            self, ReturnToBase, '/rover/return_to_base',
            callback_group=cb_group)
        self.drone_spray_client = ActionClient(
            self, SprayZone, '/drone/spray_zone',
            callback_group=cb_group)
        self.drone_return_client = ActionClient(
            self, ReturnToBase, '/drone/return_to_rover',
            callback_group=cb_group)

        # Status timer
        self.status_timer = self.create_timer(
            1.0, self.publish_status, callback_group=cb_group)

        self.get_logger().info('Mission Manager started. State: IDLE')
        self.get_logger().info('Send commands to /mission/command_raw')

    # ── State Machine ─────────────────────────────────────────────────────────

    def transition_to(self, new_state: MissionState, reason: str = ''):
        if new_state not in VALID_TRANSITIONS.get(self.state, []):
            self.get_logger().error(
                f'Invalid transition: {self.state.value} → {new_state.value}')
            return False
        old = self.state
        self.state = new_state
        self.get_logger().info(
            f'State: {old.value} → {new_state.value}'
            + (f' [{reason}]' if reason else '')
        )
        return True

    def enter_emergency(self, reason: str):
        self.last_error = reason
        self.get_logger().error(f'EMERGENCY: {reason}')
        # Force transition (bypasses normal validation)
        self.state = MissionState.EMERGENCY
        self.mission_progress = 0.0

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def command_raw_callback(self, msg: String):
        if self.state != MissionState.IDLE:
            self.get_logger().warn(
                f'Ignoring command - currently in state {self.state.value}. '
                f'Send /mission/reset to abort current mission.')
            return
        self.get_logger().info(f'Command received: "{msg.data}"')
        # Spawn async mission execution
        asyncio.ensure_future(self.execute_mission(msg.data))

    def reset_callback(self, msg: Bool):
        if msg.data:
            if self.state == MissionState.EMERGENCY:
                self.get_logger().info('Resetting from EMERGENCY to IDLE')
                self.state = MissionState.IDLE
                self.last_error = ''
            elif self.state != MissionState.IDLE:
                self.get_logger().warn('Force-resetting mission to IDLE')
                self.state = MissionState.IDLE
            else:
                self.get_logger().info('Already IDLE, nothing to reset')

    def drone_status_callback(self, msg: DroneStatus):
        self.drone_status = msg

    def rover_status_callback(self, msg: RoverStatus):
        self.rover_status = msg
        self.rover_pose.pose = msg.current_pose
        self.rover_pose.header.stamp = msg.header.stamp

    def publish_status(self):
        msg = MissionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.current_state = self.state.value
        msg.active_zone = self.active_zone
        msg.progress = self.mission_progress
        msg.status_message = self.last_error or f'State: {self.state.value}'
        msg.is_emergency = self.state == MissionState.EMERGENCY
        msg.drone_status = self.drone_status.flight_mode if self.drone_status else ''
        msg.rover_status = self.rover_status.current_zone if self.rover_status else ''
        self.status_pub.publish(msg)

    # ── Mission Execution ─────────────────────────────────────────────────────

    async def execute_mission(self, command_text: str):
        """Full mission pipeline from command to completion."""
        try:
            # 1. Parse command via LLM
            zone_info = await self.parse_command_via_llm(command_text)
            if zone_info is None:
                return

            # Handle special actions
            if zone_info.action == 'return_to_base':
                await self.return_rover_to_base()
                return
            if zone_info.action == 'abort':
                self.get_logger().info('Abort command received')
                return

            if zone_info.action != 'spray' or zone_info.zone_id not in ZONES_CONFIG:
                self.get_logger().error(
                    f'Cannot execute action={zone_info.action} zone={zone_info.zone_id}')
                self.transition_to(MissionState.IDLE, 'invalid command')
                return

            self.active_zone = zone_info.zone_id

            # 2. Navigate rover to zone
            if not await self.navigate_rover_to_zone(zone_info.zone_id):
                return

            # 3. Deploy and spray
            if not await self.deploy_and_spray(zone_info):
                return

            # 4. Return drone to rover cart
            if not await self.return_drone_to_rover():
                return

            # 5. Return rover to base
            await self.return_rover_to_base()

        except Exception as e:
            self.enter_emergency(f'Unexpected error in mission: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    async def parse_command_via_llm(self, text: str):
        """Call LLM service to parse command. Returns ZoneInfo or None on failure."""
        if not self.transition_to(MissionState.VALIDATING_COMMAND, f'"{text}"'):
            return None

        if not self.llm_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('LLM service not available')
            self.enter_emergency('LLM service unavailable')
            return None

        req = ParseMissionCommand.Request()
        req.command_text = text
        req.language_hint = 'auto'

        future = self.llm_client.call_async(req)
        await future

        if not future.result():
            self.enter_emergency('LLM service call failed')
            return None

        resp = future.result()
        self.get_logger().info(
            f'LLM result: action={resp.action} zone={resp.zone_id} '
            f'confidence={resp.confidence:.2f} - {resp.explanation}'
        )

        if not resp.success or resp.confidence < self.min_confidence:
            self.get_logger().warn(
                f'Low confidence ({resp.confidence:.2f}) or failed parse. '
                f'Returning to IDLE.')
            self.transition_to(MissionState.IDLE, 'low confidence command')
            return None

        zone_info = ZoneInfo()
        zone_info.zone_id = resp.zone_id
        zone_info.action = resp.action
        zone_info.confidence = resp.confidence
        zone_info.spray_height_m = resp.spray_height_m
        zone_info.overlap_percent = resp.overlap_percent
        zone_info.speed_ms = resp.speed_ms
        return zone_info

    async def navigate_rover_to_zone(self, zone_id: str) -> bool:
        """Send rover to zone. Returns True on success."""
        if not self.transition_to(MissionState.TRANSITING_TO_ZONE, f'→ Zone {zone_id}'):
            return False

        if not self.rover_nav_client.wait_for_server(timeout_sec=10.0):
            self.enter_emergency('Rover navigate action server not available')
            return False

        goal = ExecuteMission.Goal()
        goal.command_text = zone_id

        self.get_logger().info(f'Sending rover to zone {zone_id}...')
        send_future = self.rover_nav_client.send_goal_async(
            goal,
            feedback_callback=lambda fb: self._on_rover_nav_feedback(fb)
        )
        await send_future
        handle = send_future.result()

        if not handle.accepted:
            self.enter_emergency(f'Rover nav goal rejected for zone {zone_id}')
            return False

        result_future = handle.get_result_async()
        await result_future
        result = result_future.result().result

        if not result.success:
            self.enter_emergency(f'Rover failed to reach zone {zone_id}: {result.error_message}')
            return False

        self.get_logger().info(f'Rover arrived at zone {zone_id}')
        return True

    def _on_rover_nav_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.mission_progress = fb.progress * 0.3  # 30% of total mission
        self.get_logger().debug(f'Rover nav progress: {fb.progress:.0%}')

    async def deploy_and_spray(self, zone_info: ZoneInfo) -> bool:
        """Arm drone, take off, execute spray pattern."""
        if not self.transition_to(MissionState.DEPLOYING_DRONE, 'arming + takeoff'):
            return False

        # Generate spray waypoints
        overlap = zone_info.overlap_percent / 100.0
        waypoints = self.spray_executor.get_zone_waypoints(
            zone_info.zone_id, ZONES_CONFIG)
        if not waypoints.poses:
            self.enter_emergency('Failed to generate spray waypoints')
            return False

        if not self.drone_spray_client.wait_for_server(timeout_sec=10.0):
            self.enter_emergency('Drone spray action server not available')
            return False

        goal = SprayZone.Goal()
        goal.zone_id = zone_info.zone_id
        goal.waypoints = waypoints
        goal.altitude = zone_info.spray_height_m

        if not self.transition_to(MissionState.SPRAYING, f'zone {zone_info.zone_id}'):
            return False

        self.get_logger().info(
            f'Starting spray: zone={zone_info.zone_id}, '
            f'alt={goal.altitude:.1f}m, waypoints={len(waypoints.poses)}'
        )

        send_future = self.drone_spray_client.send_goal_async(
            goal,
            feedback_callback=lambda fb: self._on_spray_feedback(fb)
        )
        await send_future
        handle = send_future.result()

        if not handle.accepted:
            self.enter_emergency('Spray goal rejected by drone controller')
            return False

        result_future = handle.get_result_async()
        await result_future
        result = result_future.result().result

        if not result.success:
            self.enter_emergency(f'Spray mission failed')
            return False

        self.get_logger().info(
            f'Spray complete. Coverage: {result.actual_coverage:.0f}%')
        return True

    def _on_spray_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        # 30-70% of total mission is spray phase
        self.mission_progress = 0.3 + fb.coverage_percent / 100.0 * 0.4
        self.get_logger().info(
            f'Spray: strip {fb.strips_completed}/{fb.total_strips}, '
            f'{fb.coverage_percent:.0f}%'
        )

    async def return_drone_to_rover(self) -> bool:
        """Return drone to rover cart via ArUco precision landing."""
        if not self.transition_to(MissionState.DRONE_RETURNING, 'ArUco landing'):
            return False

        if not self.drone_return_client.wait_for_server(timeout_sec=10.0):
            self.enter_emergency('Drone return action server not available')
            return False

        goal = ReturnToBase.Goal()
        goal.target_pose = self.rover_pose
        goal.use_aruco_landing = True

        send_future = self.drone_return_client.send_goal_async(
            goal,
            feedback_callback=lambda fb: self._on_drone_return_feedback(fb)
        )
        await send_future
        handle = send_future.result()

        if not handle.accepted:
            self.enter_emergency('Drone return goal rejected')
            return False

        result_future = handle.get_result_async()
        await result_future
        result = result_future.result().result

        if not result.success:
            self.enter_emergency('Drone failed to land on rover cart')
            return False

        self.get_logger().info(
            f'Drone landed on cart. ArUco used: {result.aruco_landing_used}')
        return True

    def _on_drone_return_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.mission_progress = 0.7 + (1.0 - fb.distance_remaining / 20.0) * 0.15
        self.get_logger().info(
            f'Drone returning: {fb.landing_phase}, dist={fb.distance_remaining:.1f}m')

    async def return_rover_to_base(self) -> bool:
        """Return rover to shed/base."""
        if self.state not in (MissionState.DRONE_RETURNING, MissionState.IDLE,
                               MissionState.TRANSITING_TO_ZONE):
            if not self.transition_to(MissionState.RETURNING_TO_BASE, 'heading home'):
                return False
        else:
            self.state = MissionState.RETURNING_TO_BASE

        if not self.rover_return_client.wait_for_server(timeout_sec=10.0):
            self.enter_emergency('Rover return action server not available')
            return False

        goal = ReturnToBase.Goal()
        goal.use_aruco_landing = False

        send_future = self.rover_return_client.send_goal_async(goal)
        await send_future
        handle = send_future.result()

        if not handle.accepted:
            self.enter_emergency('Rover return goal rejected')
            return False

        result_future = handle.get_result_async()
        await result_future
        result = result_future.result().result

        if result.success:
            self.get_logger().info('Mission complete! Rover back at base.')
            self.mission_progress = 1.0
            self.transition_to(MissionState.IDLE, 'mission complete')
            return True
        else:
            self.enter_emergency('Rover failed to return to base')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
