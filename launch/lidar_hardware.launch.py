#!/usr/bin/env python3
"""
Real Hardware LiDAR Launch
----------------------------
Starts the RPLiDAR driver for physical robot.

SUPPORTED LIDARS (change 'executable' below):
  RPLiDAR A1/A2/A3 : rplidar_composition  (package: rplidar_ros)
  YDLiDAR          : ydlidar_ros2_driver  (package: ydlidar_ros2_driver)
  Hokuyo           : urg_node            (package: urg_node)

FIND YOUR PORT:
  ls /dev/ttyUSB* /dev/ttyACM*
  OR: dmesg | grep tty   (after plugging LiDAR)
  Common: /dev/ttyUSB0 (RPLiDAR), /dev/ttyACM0 (some others)

INSTALL DRIVER (if not installed):
  sudo apt install ros-humble-rplidar-ros

USAGE (standalone):
  ros2 launch ackerman_pkg lidar_hardware.launch.py
  
VERIFY IT WORKS:
  ros2 topic echo /scan
  ros2 run rviz2 rviz2   (add LaserScan display → topic /scan)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # ── Launch arguments (can override at command line) ──
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for LiDAR (e.g. /dev/ttyUSB0 or /dev/ttyACM0)'
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_frame',
        description='TF frame ID for LiDAR scans'
    )

    serial_port = LaunchConfiguration('serial_port')
    frame_id = LaunchConfiguration('frame_id')

    # ── RPLiDAR Driver Node ──
    # Change this node if using a different LiDAR model
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'frame_id': frame_id,
            'angle_compensate': True,
            'scan_mode': 'Standard',
            'serial_baudrate': 115200,
        }],
        # Remap output to standard /scan topic (Nav2 expects this)
        remappings=[
            ('/scan', '/scan'),
        ]
    )

    return LaunchDescription([
        serial_port_arg,
        frame_id_arg,
        lidar_node,
    ])
