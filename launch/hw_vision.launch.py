#!/usr/bin/env python3
"""
hw_vision.launch.py
--------------------
All vision + sensor fusion + lane follower nodes for real hardware.

Reads:  /image        (from hw_camera.launch.py)
        /scan         (from hw_lidar.launch.py)
        /cmd_vel_nav  (from Nav2)

Writes: /cmd_vel      (final motor commands to robot hardware)
        /obstacles_fused, /lane_offset_fused, /risk_level, /speed_factor

Standalone (for vision testing only):
  ros2 launch ackerman_pkg hw_vision.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('ackerman_pkg')
    vision_config = os.path.join(pkg_dir, 'config', 'vision_hardware.yaml')

    # ── 1. Object Detection (YOLOv8n) ───────────────────────────────────────
    object_detection = Node(
        package='ackerman_pkg',
        executable='object_detection_node.py',
        name='object_detection_node',
        output='screen',
        parameters=[
            vision_config,
            {'camera_topic': '/image'},
            {'confidence_threshold': '0.5'},
            {'enable_visualization': 'true'},
            {'use_sim_time': False},
        ],
    )

    # ── 2. Lane Detection (HSV) ──────────────────────────────────────────────
    lane_detection = Node(
        package='ackerman_pkg',
        executable='lane_detection_node.py',
        name='lane_detection_node',
        output='screen',
        parameters=[
            vision_config,
            {'camera_topic': '/image'},
            {'enable_visualization': 'true'},
            {'use_sim_time': False},
        ],
    )

    # ── 3. Sensor Fusion (binary risk: camera → risk 1.0 EMERGENCY) ─────────
    sensor_fusion = Node(
        package='ackerman_pkg',
        executable='sensor_fusion_node.py',
        name='sensor_fusion_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'use_camera_obstacles': 'true',
            'use_lane_following': 'true',
            'lidar_angle_range': '180.0',
            'lidar_min_safe_distance': '0.5',
            'fusion_rate': '10.0',
        }],
    )

    # ── 4. Lane Follower (risk-gated controller) ─────────────────────────────
    # IN:  /cmd_vel_nav (Nav2 commands)
    # OUT: /cmd_vel     (to robot motors)
    lane_follower = Node(
        package='ackerman_pkg',
        executable='lane_follower_node.py',
        name='lane_follower_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'enable_lane_following': 'true',
            'lane_offset_gain': '0.5',
            'max_steering_angle': '0.5',
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),             # IN: from Nav2
            ('/cmd_vel_lane_corrected', '/cmd_vel'),  # OUT: to robot hardware
        ],
    )

    return LaunchDescription([
        object_detection,
        lane_detection,
        sensor_fusion,
        lane_follower,
    ])
