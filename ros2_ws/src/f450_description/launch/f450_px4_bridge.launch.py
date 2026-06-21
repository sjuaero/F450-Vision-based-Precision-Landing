#!/usr/bin/env python3
"""
PX4 SITL 전용 ROS2 브리지 + 착륙패드 감지 launch.

실행 순서:
  터미널 1: ~/PX4-Autopilot/run_drone_landing.sh
  터미널 2: MicroXRCEAgent udp4 -p 8888
  터미널 3: ros2 launch f450_description f450_px4_bridge.launch.py
  터미널 4: python3 ~/scripts/drone_mission.py

이 파일이 실행하는 것:
  1. image_bridge   : Gazebo /f450/camera/image_raw → ROS2
  2. ros_gz_bridge  : /f450/camera/camera_info, /clock → ROS2
  3. landing_pad_detector: 착륙패드 ArUco(ID 0-3) 감지
       → /landing/state (SEARCHING/DETECTED/ALIGNED)
       → /landing/offset (패드 중심 오프셋 m)
       → /landing/altitude (추정 고도 m)
       → /landing/debug_image (디버그 영상)
  4. rqt_image_view : /landing/debug_image 표시
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_f450   = get_package_share_directory('f450_description')
    bridge_yaml = os.path.join(pkg_f450, 'config', 'ros_gz_bridge.yaml')

    # 1. Gazebo 카메라 이미지 → ROS2 (image_bridge가 더 안정적)
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='image_bridge',
        arguments=['/f450/camera/image_raw'],
        output='screen',
    )

    # 2. camera_info / clock 브리지 (ros_gz_bridge.yaml 사용)
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_yaml}],
        output='screen',
    )

    # 3. 착륙패드 감지 노드 (3초 후 - 브리지 준비 대기)
    landing_pad_detector = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='agri_aruco_detector',
                executable='landing_pad_detector',
                name='landing_pad_detector',
                output='screen',
                parameters=[{
                    'use_sim_time':        True,
                    'camera_topic':        '/f450/camera/image_raw',
                    'camera_info_topic':   '/f450/camera/camera_info',
                    'marker_size_m':       0.12,   # landing_pad_charuco 내 개별 마커 크기
                    'align_threshold_m':   0.10,
                    'publish_debug_image': True,
                }],
            )
        ]
    )

    # 4. 카메라 뷰 (5초 후 - 감지 노드 준비 대기)
    rqt_view = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rqt_image_view',
                executable='rqt_image_view',
                name='rqt_image_view',
                arguments=['/landing/debug_image'],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        image_bridge,
        ros_gz_bridge,
        landing_pad_detector,
        rqt_view,
    ])
