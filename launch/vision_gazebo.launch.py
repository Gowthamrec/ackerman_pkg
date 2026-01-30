#!/usr/bin/env python3
"""
Vision System Launch File for Gazebo Simulation
"""

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('ackerman_pkg')
    
    # Object Detection Node
    object_detection_node = Node(
        package='ackerman_pkg',
        executable='object_detection_node.py',
        name='object_detection',
        output='screen',
        parameters=[
            {'camera_topic': '/camera/image_raw'},
            {'model_path': 'yolov8n.pt'},
            {'confidence_threshold': '0.5'},
            {'enable_visualization': 'true'},
        ]
    )
    
    # Lane Detection Node
    lane_detection_node = Node(
        package='ackerman_pkg',
        executable='lane_detection_node.py',
        name='lane_detection',
        output='screen',
        parameters=[
            {'camera_topic': '/camera/image_raw'},
            {'enable_visualization': 'true'},
            {'lower_hsv': [0, 0, 200]},
            {'upper_hsv': [180, 30, 255]},
        ]
    )
    
    # Visual Odometry Node (disabled for Gazebo - lidar SLAM preferred)
    visual_odometry_node = Node(
        package='ackerman_pkg',
        executable='visual_odometry_node.py',
        name='visual_odometry',
        output='screen',
        parameters=[
            {'camera_topic': '/camera/image_raw'},
            {'publish_tf': 'false'},
        ]
    )
    
    return LaunchDescription([
        object_detection_node,
        lane_detection_node,
        # visual_odometry_node,  # Uncomment to enable in Gazebo
    ])
