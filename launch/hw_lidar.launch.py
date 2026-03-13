#!/usr/bin/env python3
"""
hw_lidar.launch.py
------------------
Launches the RPLidar hardware driver.
Publishes: /scan (sensor_msgs/LaserScan)

Standalone test:
  ros2 launch ackerman_pkg hw_lidar.launch.py
  ros2 topic echo /scan
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='USB port for RPLidar'
    )
    baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='Baud rate for RPLidar'
    )
    frame_id_arg = DeclareLaunchArgument(
        'lidar_frame',
        default_value='laser',
        description='TF frame id for LiDAR'
    )

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
            'frame_id': LaunchConfiguration('lidar_frame'),
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }]
    )

    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        frame_id_arg,
        rplidar_node,
    ])
