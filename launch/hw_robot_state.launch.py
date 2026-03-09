#!/usr/bin/env python3
"""
hw_robot_state.launch.py
------------------------
Publishes robot URDF to /robot_description and broadcasts
static TF tree (base_link → wheels, camera_link, laser, etc.)

This replaces what Gazebo does automatically in simulation.

Publishes:
  /robot_description  (String)
  /tf_static          (full TF tree from URDF)

Standalone test:
  ros2 launch ackerman_pkg hw_robot_state.launch.py
  ros2 topic echo /robot_description --no-arr
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('ackerman_pkg')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'my_bot.urdf.xacro')

    # Robot State Publisher — reads URDF, publishes TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_file]),
                value_type=str
            ),
            'use_sim_time': False,
        }]
    )

    # Joint State Publisher — needed for wheel joints (no Gazebo on HW)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # Static TF: base_footprint → base_link  (if not in URDF)
    static_tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        parameters=[{'use_sim_time': False}]
    )

    # Static TF: base_link → laser  (mount your LiDAR position here)
    # Adjust x, y, z, roll, pitch, yaw to match your physical mount
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['0.1', '0', '0.15', '0', '0', '0', 'base_link', 'laser'],
        parameters=[{'use_sim_time': False}]
    )

    # Static TF: base_link → camera_link  (mount your camera position here)
    # Adjust x, y, z, roll, pitch, yaw to match your physical mount
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera',
        arguments=['0.15', '0', '0.12', '0', '0', '0', 'base_link', 'camera_link'],
        parameters=[{'use_sim_time': False}]
    )

    # Visual Odometry — publishes /odom and TF odom → base_footprint
    # Used instead of wheel encoders on real hardware
    visual_odometry = Node(
        package='ackerman_pkg',
        executable='visual_odometry_node.py',
        name='visual_odometry_node',
        output='screen',
        parameters=[{
            'camera_topic': '/image',
            'publish_tf': 'true',     # MUST be true on hardware (no Gazebo)
            'camera_fx': '500.0',     # ← replace with your calibrated value
            'camera_fy': '500.0',     # ← replace with your calibrated value
            'camera_cx': '320.0',     # ← image_width / 2  (640/2)
            'camera_cy': '240.0',     # ← image_height / 2 (480/2)
            'camera_height': '0.15',  # ← measure: height of camera above ground (metres)
            'min_features': '30',
            'use_sim_time': False,
        }]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        static_tf_base,
        static_tf_lidar,
        static_tf_camera,
        visual_odometry,
    ])
