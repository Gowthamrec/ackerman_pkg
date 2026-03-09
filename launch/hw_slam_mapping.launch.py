#!/usr/bin/env python3
"""
hw_slam_mapping.launch.py
--------------------------
SLAM Toolbox in MAPPING mode for real hardware.
Drive the robot around → map builds live → save when done.

Prerequisites (run first):
  ros2 launch ackerman_pkg hw_robot_state.launch.py
  ros2 launch ackerman_pkg hw_lidar.launch.py
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

Save map when done:
  ros2 run nav2_map_server map_saver_cli \
    -f ~/car_project_ws/src/ackerman_pkg/map/my_map

Standalone:
  ros2 launch ackerman_pkg hw_slam_mapping.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('ackerman_pkg')
    default_params = os.path.join(pkg_dir, 'config', 'slam_hardware.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='SLAM toolbox parameters file'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': False},
        ],
    )

    return LaunchDescription([
        params_arg,
        slam_node,
    ])
