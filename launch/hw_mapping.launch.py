#!/usr/bin/env python3
"""
hw_mapping.launch.py
--------------------
Convenience launch for REAL HARDWARE MAPPING SESSION.
Run this to build a map — drive robot around with teleop.

Includes:
  1. hw_robot_state  → URDF + TF + Visual Odometry (/odom)
  2. hw_lidar        → RPLidar → /scan
  3. hw_camera       → USB webcam → /image  (needed for visual odom)
  4. hw_slam_mapping → SLAM Toolbox in mapping mode → builds /map

After mapping, save the map:
  ros2 run nav2_map_server map_saver_cli \
    -f ~/car_project_ws/src/ackerman_pkg/map/my_map

Usage:
  ros2 launch ackerman_pkg hw_mapping.launch.py
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir   = get_package_share_directory('ackerman_pkg')
    launch_dir = os.path.join(pkg_dir, 'launch')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='USB port for RPLidar'
    )
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='USB webcam device'
    )

    robot_state = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'hw_robot_state.launch.py')
        )
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'hw_lidar.launch.py')
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port'),
        }.items()
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'hw_camera.launch.py')
        ),
        launch_arguments={
            'camera_device': LaunchConfiguration('camera_device'),
        }.items()
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'hw_slam_mapping.launch.py')
        )
    )

    banner = LogInfo(msg=[
        '\n',
        '=======================================================\n',
        '   REAL HARDWARE MAPPING SESSION STARTED\n',
        '=======================================================\n',
        '  Odometry source : Visual Odometry (camera)\n',
        '  LiDAR port      : ', LaunchConfiguration('serial_port'), '\n',
        '  Camera device   : ', LaunchConfiguration('camera_device'), '\n',
        '-------------------------------------------------------\n',
        '  Drive with: ros2 run teleop_twist_keyboard teleop_twist_keyboard\n',
        '  In RViz: Fixed Frame = map, add Map topic = /map\n',
        '  Drive slowly — camera needs features to track motion\n',
        '-------------------------------------------------------\n',
        '  Save map when done:\n',
        '  ros2 run nav2_map_server map_saver_cli \\\n',
        '    -f ~/car_project_ws/src/ackerman_pkg/map/my_map\n',
        '=======================================================\n',
    ])

    return LaunchDescription([
        serial_port_arg,
        camera_device_arg,
        banner,
        robot_state,   # TF tree + visual odom
        lidar,         # /scan
        camera,        # /image
        slam,          # map building
    ])
