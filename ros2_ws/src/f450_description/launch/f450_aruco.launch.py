#!/usr/bin/env python3
"""
F450 + ArUco 마커 인식 통합 런치파일 (최종)
mesh_path를 xacro 인자로 주입 → Gazebo file:// 절대경로 사용
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess,
    IncludeLaunchDescription, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_f450   = get_package_share_directory('f450_description')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')

    xacro_file  = os.path.join(pkg_f450, 'urdf', 'f450.urdf.xacro')
    world_file  = os.path.join(pkg_f450, 'worlds', 'aruco_landing.sdf')
    # 메쉬 절대경로 → xacro 인자로 주입
    mesh_path   = os.path.join(pkg_f450, 'meshes')
    models_path = os.path.join(pkg_f450, 'models')
    bridge_yaml = os.path.join(pkg_f450, 'config', 'ros_gz_bridge.yaml')

    # Gazebo 모델 탐색 경로 (ArUco 마커 모델용)
    gz_resource = models_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')

    # XACRO → URDF  (mesh_path 인자 주입)
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' mesh_path:=', mesh_path]),
        value_type=str
    )

    # ── Launch Arguments ──────────────────────────────────
    marker_id_arg   = DeclareLaunchArgument('marker_id',   default_value='0')
    marker_size_arg = DeclareLaunchArgument('marker_size', default_value='0.8')

    # ── 1. Gazebo Harmonic ────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file, '--verbose', '1'],
        output='screen',
        additional_env={'GZ_SIM_RESOURCE_PATH': gz_resource}
    )

    # ── 2. robot_state_publisher ──────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    # ── 3. F450 스폰 (3초 대기) ───────────────────────────
    spawn = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_ros_gz, 'launch', 'gz_spawn_model.launch.py')
                ),
                launch_arguments={
                    'world':  'aruco_landing',
                    'topic':  '/robot_description',
                    'entity_name': 'f450',
                    'x': '0.0', 'y': '0.0', 'z': '0.5',
                }.items()
            )
        ]
    )

    # ── 4. Image 브릿지 ───────────────────────────────────
    image_bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_image',
                executable='image_bridge',
                name='image_bridge',
                arguments=['/f450/camera/image_raw'],
                output='screen',
            )
        ]
    )

    # ── 5. 나머지 토픽 브릿지 (CameraInfo, IMU 등) ────────
    bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge',
                output='screen',
                parameters=[{'config_file': bridge_yaml}]
            )
        ]
    )

    # ── 6. ArUco 마커 검출 노드 ───────────────────────────
    aruco_detector = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='agri_aruco_detector',
                executable='aruco_detector',
                name='aruco_detector',
                output='screen',
                parameters=[{
                    'use_sim_time':        True,
                    'camera_topic':        '/f450/camera/image_raw',
                    'camera_info_topic':   '/f450/camera/camera_info',
                    'marker_id':           LaunchConfiguration('marker_id'),
                    'marker_size_m':       LaunchConfiguration('marker_size'),
                    'aruco_dict':          'DICT_4X4_50',
                    'publish_debug_image': True,
                }]
            )
        ]
    )

    # 7. 착륙 패드 검출 노드 (4-마커 ChArUco 패드 -> 고도/오프셋 추정)
    landing_pad_detector = TimerAction(
        period=8.0,
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
                    'marker_size_m':       0.12,
                    'align_threshold_m':   0.10,
                    'publish_debug_image': True,
                }]
            )
        ]
    )

    return LaunchDescription([
        marker_id_arg,
        marker_size_arg,
        gazebo,
        rsp,
        spawn,
        image_bridge,
        bridge,
        aruco_detector,
        landing_pad_detector,
    ])
