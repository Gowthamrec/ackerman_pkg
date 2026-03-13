#!/usr/bin/env python3
"""
hw_localization.launch.py
--------------------------
AMCL localization on a saved map for real hardware.
Use this AFTER mapping is done and map is saved.

Prerequisites (run first):
  ros2 launch ackerman_pkg hw_robot_state.launch.py
  ros2 launch ackerman_pkg hw_lidar.launch.py

Usage:
  ros2 launch ackerman_pkg hw_localization.launch.py \
        map:=/home/surjith/car_project/src/ackerman_pkg/map/my_map.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('ackerman_pkg')
    default_map = os.path.join(pkg_dir, 'map', 'my_map.yaml')
    default_nav2_params = os.path.join(pkg_dir, 'config', 'nav2.yaml')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to saved map .yaml file'
    )
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_nav2_params,
        description='Nav2 parameters file (contains AMCL params)'
    )

    # Map server — loads the saved .pgm map
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': LaunchConfiguration('map'),
        }]
    )

    # AMCL — localizes robot on the loaded map using /scan
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': False},
        ]
    )

    # Lifecycle manager for map_server + amcl
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }]
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        map_server,
        amcl,
        lifecycle_manager,
    ])
