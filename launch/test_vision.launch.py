#!/usr/bin/env python3
"""
Test launch file for vision nodes without RViz (avoid display server issues)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'ackerman_pkg'
    pkg_path = get_package_share_directory(package_name)
    
    # Robot state publisher
    xacro_file = os.path.join(pkg_path, 'urdf', 'my_bot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': True}
    
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        )
    )

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot', '-z', '0.25'],
        output='screen'
    )

    # Include vision system
    vision_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_path, 'launch', 'vision_gazebo.launch.py'
        )])
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        vision_gazebo,
    ])
