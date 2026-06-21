#!/usr/bin/env python3
"""F450 드론 단독 스폰 런치파일 (Gazebo Harmonic + ROS2 Jazzy)"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('f450_description')

    xacro_file = os.path.join(pkg, 'urdf', 'f450.urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', 'empty.sdf')
    mesh_path  = os.path.join(pkg, 'meshes')

    # XACRO → URDF (mesh 절대경로 주입)
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' mesh_path:=', mesh_path]),
        value_type=str
    )

    # 1. Gazebo Harmonic 실행
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file, '--verbose', '1'],
        output='screen'
    )

    # 2. robot_state_publisher (URDF를 /robot_description 토픽으로 발행)
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

    # 3. F450 스폰 (Gazebo가 뜰 때까지 3초 대기)
    spawn = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_f450',
                output='screen',
                parameters=[{
                    'world':       'empty',
                    'topic':       '/robot_description',
                    'entity_name': 'f450',
                    'x':  '0.0',
                    'y':  '0.0',
                    'z':  '0.5',
                    'R':  '0.0',
                    'P':  '0.0',
                    'Y':  '0.0',
                }]
            )
        ]
    )

    return LaunchDescription([gazebo, rsp, spawn])
