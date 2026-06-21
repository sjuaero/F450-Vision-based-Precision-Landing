#!/usr/bin/env python3
"""
Precision Landing Launch File.

PX4 SITL + Gazebo는 별도로 실행:
  ~/PX4-Autopilot/run_drone_landing.sh

이 파일은 다음만 실행:
  1. ros_gz_bridge  (Gazebo camera/clock → ROS2)
  2. MAVROS         (ROS2 ↔ PX4 SITL MAVLink)
  3. ArUco Detector (하향 카메라 → /aruco/pose)
  4. Drone Controller (정밀착륙 액션 서버)

사용법:
  source ~/ros2_ws/install/setup.bash
  ros2 launch agri_bringup precision_landing.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    agri_gazebo_dir  = get_package_share_directory('agri_gazebo')
    agri_drone_dir   = get_package_share_directory('agri_drone_controller')
    agri_bringup_dir = get_package_share_directory('agri_bringup')

    bridge_config  = os.path.join(agri_gazebo_dir,  'config', 'ros_gz_bridge.yaml')
    mavros_params  = os.path.join(agri_drone_dir,   'config', 'mavros_params.yaml')
    mission_config = os.path.join(agri_bringup_dir, 'config', 'mission_config.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ── 1. ros_gz_bridge: Gazebo 카메라 + clock → ROS2 ───────────────────
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    # ── 2. MAVROS: ROS2 ↔ PX4 SITL (instance 1, port 14541/14581) ───────
    # fcu_url은 mavros_params.yaml에서 읽음: udp://:14541@localhost:14581
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        namespace='',
        parameters=[mavros_params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ── 3. ArUco Detector ────────────────────────────────────────────────
    # camera_info 없어도 f450 SDF intrinsics fallback으로 즉시 동작
    aruco_detector = Node(
        package='agri_aruco_detector',
        executable='aruco_detector',
        name='aruco_detector',
        parameters=[
            mission_config,
            {'use_sim_time': use_sim_time},
            {
                'camera_topic':      '/drone/camera/image_raw',
                'camera_info_topic': '/drone/camera/camera_info',
                'marker_id':         42,
                'marker_size_m':     0.5,
                'aruco_dict':        'DICT_4X4_50',
                'publish_debug_image': True,
                # f450 SDF 실제 intrinsics
                'fallback_fx': 662.62,
                'fallback_fy': 618.45,
                'fallback_cx': 409.89,
                'fallback_cy': 220.37,
                'fallback_frame_id': 'camera_optical_frame',
            },
        ],
        output='screen',
    )

    # ── 4. Drone Controller ───────────────────────────────────────────────
    drone_controller = Node(
        package='agri_drone_controller',
        executable='drone_controller',
        name='drone_controller',
        parameters=[
            mission_config,
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # bridge 먼저, 나머지는 2초 후 (Gazebo가 이미 떠 있다고 가정)
        ros_gz_bridge,
        TimerAction(period=2.0, actions=[mavros_node]),
        TimerAction(period=3.0, actions=[aruco_detector]),
        TimerAction(period=3.0, actions=[drone_controller]),
    ])
