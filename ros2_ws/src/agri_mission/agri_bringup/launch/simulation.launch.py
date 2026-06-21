#!/usr/bin/env python3
"""
Full simulation launch file.

Starts:
1. Gazebo Harmonic with agricultural world
2. ros_gz_bridge (Gazebo <-> ROS2 topic bridge)
3. MAVROS2 (PX4 SITL drone control)
4. All mission system nodes

Usage:
  ros2 launch agri_bringup simulation.launch.py
  ros2 launch agri_bringup simulation.launch.py use_nav2:=false  # skip Nav2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    ExecuteProcess, GroupAction, SetEnvironmentVariable, TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directories
    agri_gazebo_dir = get_package_share_directory('agri_gazebo')
    agri_bringup_dir = get_package_share_directory('agri_bringup')
    agri_drone_dir = get_package_share_directory('agri_drone_controller')
    agri_rover_dir = get_package_share_directory('agri_rover_controller')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_nav2 = LaunchConfiguration('use_nav2', default='true')
    use_mavros = LaunchConfiguration('use_mavros', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='false')

    world_file = os.path.join(agri_gazebo_dir, 'worlds', 'agricultural_field.sdf')
    bridge_config = os.path.join(agri_gazebo_dir, 'config', 'ros_gz_bridge.yaml')
    mission_config = os.path.join(agri_bringup_dir, 'config', 'mission_config.yaml')
    nav2_params = os.path.join(agri_rover_dir, 'config', 'nav2_params.yaml')
    mavros_params = os.path.join(agri_drone_dir, 'config', 'mavros_params.yaml')

    # ── Gazebo Harmonic ──────────────────────────────────────────────
    gz_env = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(agri_gazebo_dir, 'models')
        + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file, '--verbose', '1'],
        output='screen',
        additional_env={'GZ_SIM_RESOURCE_PATH': os.path.join(agri_gazebo_dir, 'models')}
    )

    # ── ros_gz_bridge (Gazebo <-> ROS2) ─────────────────────────────
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    # ── Spawn rover model in Gazebo ──────────────────────────────────
    spawn_rover = TimerAction(
        period=3.0,  # Wait for Gazebo to fully start
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service', '-s', '/world/agricultural_field/create',
                    '--reqtype', 'gz.msgs.EntityFactory',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '5000',
                    '--req',
                    f'sdf_filename: "{os.path.join(agri_gazebo_dir, "models", "rover_with_cart", "model.sdf")}"'
                    ' name: "rover_with_cart"'
                    ' pose { position { x: 0 y: 0 z: 0.1 } }',
                ],
                output='screen',
            )
        ]
    )

    # ── MAVROS2 (drone <-> PX4 SITL) ────────────────────────────────
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        namespace='',
        parameters=[mavros_params, {'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_mavros),
    )

    # ── Nav2 (rover navigation) ──────────────────────────────────────
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
        }.items(),
        condition=IfCondition(use_nav2),
    )

    # ── Mission System Nodes ─────────────────────────────────────────

    llm_node = Node(
        package='agri_llm_interface',
        executable='llm_node',
        name='llm_interface',
        parameters=[
            mission_config,
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    aruco_detector = Node(
        package='agri_aruco_detector',
        executable='aruco_detector',
        name='aruco_detector',
        parameters=[
            mission_config,
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

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

    rover_controller = Node(
        package='agri_rover_controller',
        executable='rover_controller',
        name='rover_controller',
        parameters=[
            mission_config,
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    mission_manager = Node(
        package='agri_mission_manager',
        executable='mission_manager',
        name='mission_manager',
        parameters=[
            mission_config,
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_nav2', default_value='true'),
        DeclareLaunchArgument('use_mavros', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),

        # Environment
        gz_env,

        # Simulation
        gazebo,
        TimerAction(period=2.0, actions=[ros_gz_bridge]),
        spawn_rover,

        # Drone interface (MAVROS -> PX4 SITL)
        TimerAction(period=5.0, actions=[mavros_node]),

        # Rover navigation
        TimerAction(period=5.0, actions=[nav2_bringup]),

        # Mission nodes (after simulation is ready)
        TimerAction(period=8.0, actions=[
            llm_node,
            aruco_detector,
            drone_controller,
            rover_controller,
            mission_manager,
        ]),
    ])
