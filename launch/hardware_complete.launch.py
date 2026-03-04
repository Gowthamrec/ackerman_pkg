#!/usr/bin/env python3
"""
Complete Real Hardware Launch
-----------------------------
Launches ALL nodes needed for real-time autonomous operation on physical robot:
  - Object detection (YOLOv8n, real camera)
  - Lane detection (HSV, real camera)
  - Sensor fusion (LiDAR + Camera + Lane → risk score)
  - Lane follower (risk-gated steering + speed scaling)

USAGE:
  Terminal 1 (robot hardware bringup - your robot driver):
    ros2 launch <your_robot_pkg> bringup.launch.py

  Terminal 2 (navigation stack):
    ros2 launch ackerman_pkg nav2.launch.py

  Terminal 3 (this file - vision + fusion + control):
    ros2 launch ackerman_pkg hardware_complete.launch.py

  Terminal 4 (optional visualization):
    ros2 launch ackerman_pkg my_bot_rviz.launch.py

REQUIRED TOPICS FROM ROBOT HARDWARE:
  /scan            → LaserScan (from LiDAR driver)
  /camera/image_raw → Image    (from camera driver)
  /cmd_vel         → Twist     (consumed from Nav2, output goes back)
  /odom            → Odometry  (from wheel encoders)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('ackerman_pkg')
    vision_config = os.path.join(pkg_dir, 'config', 'vision_hardware.yaml')

    # ── 1. Object Detection (YOLOv8n) ──────────────────────────────────────
    # Reads: /camera/image_raw
    # Writes: /detections, /obstacles_detected, /object_detection/image
    object_detection = Node(
        package='ackerman_pkg',
        executable='object_detection_node.py',
        name='object_detection_node',
        parameters=[
            vision_config,
            {'camera_topic': '/camera/image_raw'},
            {'confidence_threshold': '0.5'},
            {'enable_visualization': 'true'},
        ],
        output='screen'
    )

    # ── 2. Lane Detection (HSV) ────────────────────────────────────────────
    # Reads: /camera/image_raw
    # Writes: /lane_status, /lane_offset, /lane_detection/image
    lane_detection = Node(
        package='ackerman_pkg',
        executable='lane_detection_node.py',
        name='lane_detection_node',
        parameters=[
            vision_config,
            {'camera_topic': '/camera/image_raw'},
            {'enable_visualization': 'true'},
            {'lower_hsv': [0, 0, 200]},
            {'upper_hsv': [180, 30, 255]},
        ],
        output='screen'
    )

    # ── 3. Sensor Fusion (Risk Assessment) ────────────────────────────────
    # Reads: /scan, /obstacles_detected, /lane_offset, /lane_status
    # Writes: /obstacles_fused, /lane_offset_fused, /risk_score,
    #         /risk_level, /speed_factor, /sensor_fusion_status
    #
    # OPTIMAL WEIGHTS (validated over 120 experiments):
    #   alpha=0.4 (LiDAR), beta=0.4 (Camera), gamma=0.2 (Lane)
    sensor_fusion = Node(
        package='ackerman_pkg',
        executable='sensor_fusion_node.py',
        name='sensor_fusion_node',
        parameters=[
            {'alpha_lidar': '0.4'},
            {'beta_camera': '0.4'},
            {'gamma_lane': '0.2'},
            {'emergency_threshold': '0.8'},
            {'slow_threshold': '0.5'},
            {'caution_threshold': '0.2'},
            {'lidar_max_relevant_dist': '5.0'},
            {'lidar_angle_range': '60.0'},
            {'lidar_min_safe_distance': '0.5'},
            {'use_camera_obstacles': 'true'},
            {'use_lane_following': 'true'},
            {'fusion_rate': '10.0'},
        ],
        output='screen'
    )

    # ── 4. Lane Follower (Risk-Gated Controller) ───────────────────────────
    # Reads: /cmd_vel (from Nav2), /lane_offset_fused, /speed_factor, /risk_level
    # Writes: /cmd_vel (remapped back — final motor commands)
    #
    # Remap: Nav2 → /cmd_vel_nav2 → this node → /cmd_vel → robot
    lane_follower = Node(
        package='ackerman_pkg',
        executable='lane_follower_node.py',
        name='lane_follower_node',
        parameters=[
            {'enable_lane_following': 'true'},
            {'lane_offset_gain': '0.5'},
            {'max_steering_angle': '0.5'},
            {'min_speed_factor': '0.05'},
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav2'),                   # IN: from Nav2
            ('/cmd_vel_lane_corrected', '/cmd_vel'),         # OUT: to robot hardware
        ],
        output='screen'
    )

    return LaunchDescription([
        object_detection,
        lane_detection,
        sensor_fusion,
        lane_follower,
    ])
