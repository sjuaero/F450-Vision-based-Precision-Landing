#!/usr/bin/env python3
"""
ArUco Detector Launch  (uXRCE-DDS 구성용)

PX4 SITL + MicroXRCEAgent 실행 후 이 파일을 실행:
  ros2 launch agri_bringup aruco_detector.launch.py

기능:
  1. ros_gz_bridge: Gazebo 카메라(/f450/camera/image_raw) → ROS2 + /clock
  2. aruco_detector: 하향 카메라에서 ArUco 마커 감지 → /aruco/pose, /aruco/detection_status
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    agri_gazebo_dir = get_package_share_directory('agri_gazebo')

    bridge_config = os.path.join(agri_gazebo_dir, 'config', 'ros_gz_bridge.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ── ros_gz_bridge: Gazebo 카메라 + clock → ROS2 ──────────────────────
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    # ── ArUco Detector ────────────────────────────────────────────────────
    aruco_detector = Node(
        package='agri_aruco_detector',
        executable='aruco_detector',
        name='aruco_detector',
        parameters=[
            {'use_sim_time': use_sim_time},
            {
                'camera_topic':      '/drone/camera/image_raw',
                'camera_info_topic': '/drone/camera/camera_info',
                'marker_id':         42,
                'marker_size_m':     0.5,
                'aruco_dict':        'DICT_4X4_50',
                'publish_debug_image': True,
                # f450 SDF 실제 intrinsics (camera_info 없어도 즉시 동작)
                'fallback_fx': 662.62,
                'fallback_fy': 618.45,
                'fallback_cx': 409.89,
                'fallback_cy': 220.37,
                'fallback_frame_id': 'camera_optical_frame',
            },
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        ros_gz_bridge,
        TimerAction(period=2.0, actions=[aruco_detector]),
    ])
