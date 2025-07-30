#!/usr/bin/env python3
"""
Launch file for min-jerk trajectory planner demonstration.

This launch file starts the min-jerk planner node and demo client
for high-precision trajectory planning applications.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    joint_limits_file_arg = DeclareLaunchArgument(
        'joint_limits_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('min_jerk_planner'),
            'config',
            'joint_limits.yaml'
        ]),
        description='Path to joint limits configuration file'
    )
    
    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='1000.0',
        description='Control frequency in Hz for high-precision control'
    )
    
    joint_names_arg = DeclareLaunchArgument(
        'joint_names',
        default_value="['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']",
        description='List of joint names for the robot'
    )
    
    trajectory_topic_arg = DeclareLaunchArgument(
        'trajectory_topic',
        default_value='/joint_trajectory_controller/joint_trajectory',
        description='Topic for publishing joint trajectories'
    )
    
    joint_states_topic_arg = DeclareLaunchArgument(
        'joint_states_topic',
        default_value='/joint_states',
        description='Topic for receiving joint states'
    )
    
    run_demo_arg = DeclareLaunchArgument(
        'run_demo',
        default_value='true',
        description='Whether to run the demo client'
    )
    
    # Min-jerk planner node
    min_jerk_planner_node = Node(
        package='min_jerk_planner',
        executable='min_jerk_planner_node',
        name='min_jerk_planner_node',
        output='screen',
        parameters=[{
            'joint_names': LaunchConfiguration('joint_names'),
            'control_frequency': LaunchConfiguration('control_frequency'),
            'joint_limits_file': LaunchConfiguration('joint_limits_file'),
            'default_velocity_scaling': 0.5,
            'default_acceleration_scaling': 0.5,
            'trajectory_topic': LaunchConfiguration('trajectory_topic'),
            'joint_states_topic': LaunchConfiguration('joint_states_topic'),
        }],
        remappings=[
            ('/joint_states', LaunchConfiguration('joint_states_topic')),
        ]
    )
    
    # Demo client node (conditional)
    demo_client_node = Node(
        package='min_jerk_planner',
        executable='demo_client',
        name='min_jerk_demo_client',
        output='screen',
        condition=lambda context: context.launch_configurations['run_demo'].lower() == 'true'
    )
    
    # RViz for visualization (optional)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('min_jerk_planner'),
        'config',
        'min_jerk_demo.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=lambda context: os.path.exists(rviz_config_file.perform(context))
    )
    
    # Joint state publisher (for simulation/testing)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'source_list': ['/joint_trajectory_controller/joint_states'],
            'rate': 100.0,  # High rate for precision
        }],
        remappings=[
            ('/joint_states', LaunchConfiguration('joint_states_topic')),
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        joint_limits_file_arg,
        control_frequency_arg,
        joint_names_arg,
        trajectory_topic_arg,
        joint_states_topic_arg,
        run_demo_arg,
        
        # Nodes
        min_jerk_planner_node,
        demo_client_node,
        joint_state_publisher_node,
        # rviz_node,  # Uncomment if RViz config exists
    ])