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
        -f /home/surjith/car_project/src/ackerman_pkg/map/my_map

Usage:
  ros2 launch ackerman_pkg hw_mapping.launch.py
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
        default_value='/dev/video2',
        description='USB webcam device'
    )
    arduino_port_arg = DeclareLaunchArgument(
        'arduino_port',
        default_value='/dev/ttyACM0',
        description='USB port for Arduino Mega motor controller'
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

    # Relay /cmd_vel → /cmd_vel_safe (skip obstacle_manager during mapping)
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=['/cmd_vel', '/cmd_vel_safe'],
        output='screen',
    )

    # Arduino bridge — translates /cmd_vel_safe → serial commands to motors
    arduino_bridge = Node(
        package='ackerman_pkg',
        executable='arduino_bridge_node.py',
        name='arduino_bridge_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('arduino_port'),
            'baud_rate': '115200',
            'use_sim_time': False,
        }]
    )

    banner = LogInfo(msg=[
        '\n',
        '=======================================================\n',
        '   REAL HARDWARE MAPPING SESSION STARTED\n',
        '=======================================================\n',
        '  Odometry source : Visual Odometry (camera)\n',
        '  LiDAR port      : ', LaunchConfiguration('serial_port'), '\n',
        '  Camera device   : ', LaunchConfiguration('camera_device'), '\n',
        '  Arduino port    : ', LaunchConfiguration('arduino_port'), '\n',
        '-------------------------------------------------------\n',
        '  Drive with: ros2 run teleop_twist_keyboard teleop_twist_keyboard\n',
        '  In RViz: Fixed Frame = map, add Map topic = /map\n',
        '  Drive slowly — camera needs features to track motion\n',
        '-------------------------------------------------------\n',
        '  Save map when done:\n',
        '  ros2 run nav2_map_server map_saver_cli \\\n',
        '    -f /home/surjith/car_project/src/ackerman_pkg/map/my_map\n',
        '=======================================================\n',
    ])

    return LaunchDescription([
        serial_port_arg,
        camera_device_arg,
        arduino_port_arg,
        banner,
        robot_state,   # TF tree + visual odom
        lidar,         # /scan
        camera,        # /image
        slam,          # map building
        cmd_vel_relay, # /cmd_vel → /cmd_vel_safe
        arduino_bridge,# serial motor commands
    ])
