#!/usr/bin/env python3
"""
Autonomous Navigation — LiDAR Only
-------------------------------------
Full autonomous navigation using ONLY LiDAR (no camera, no lane detection).
Robot uses Nav2 + AMCL + LiDAR costmaps to navigate.
Obstacle manager adds smart 5-second stop + replan logic.
Arduino bridge sends final commands to hardware.

DATA FLOW:
  /scan (LiDAR)
      │
      ├──► Nav2 (costmap + path planning)
      │         └── /cmd_vel_nav2
      │
      ├──► sensor_fusion_node (LiDAR risk only, camera/lane disabled)
      │         └── /risk_score, /risk_level, /speed_factor
      │
      └──► lane_follower_node (speed scaling only, no steering correction)
                └── /cmd_vel
                      │
                      └──► obstacle_manager_node (5s stop + replan)
                                  └── /cmd_vel_safe
                                          │
                                          └──► arduino_bridge_node
                                                      │
                                                      └──► Arduino Mega
                                                                │
                                                        Motors + Steering

USAGE:
  Terminal 1 — LiDAR driver:
    ros2 launch ackerman_pkg lidar_hardware.launch.py serial_port:=/dev/ttyUSB0

  Terminal 2 — Localization (choose one):
    ros2 launch ackerman_pkg slam.launch.py          ← if building map
    ros2 launch ackerman_pkg localization.launch.py  ← if map exists

  Terminal 3 — Navigation:
    ros2 launch ackerman_pkg nav2.launch.py

  Terminal 4 — THIS FILE (autonomous control):
    ros2 launch ackerman_pkg autonomous_lidar_only.launch.py

  Terminal 5 — Visualization (optional):
    ros2 launch ackerman_pkg my_bot_rviz.launch.py
    Then use "2D Nav Goal" in RViz to set destination.

TROUBLESHOOT:
  ros2 topic echo /scan               ← verify LiDAR data
  ros2 topic echo /obstacle_status    ← see obstacle manager state
  ros2 topic echo /arduino_status     ← verify Arduino connection
  ros2 topic echo /cmd_vel_safe       ← final motor commands
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # ── Launch arguments ──
    arduino_port_arg = DeclareLaunchArgument(
        'arduino_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino Mega'
    )
    critical_range_arg = DeclareLaunchArgument(
        'critical_range',
        default_value='0.30',
        description='Critical stop distance in meters'
    )

    arduino_port = LaunchConfiguration('arduino_port')
    critical_range = LaunchConfiguration('critical_range')

    # ── 1. Sensor Fusion (LiDAR only mode) ────────────────────────
    # Camera and lane disabled → pure LiDAR risk
    sensor_fusion = Node(
        package='ackerman_pkg',
        executable='sensor_fusion_node.py',
        name='sensor_fusion_node',
        parameters=[
            {'alpha_lidar': '1.0'},       # 100% LiDAR weight
            {'beta_camera': '0.0'},       # Camera disabled
            {'gamma_lane': '0.0'},        # Lane disabled
            {'emergency_threshold': '0.8'},
            {'slow_threshold': '0.5'},
            {'caution_threshold': '0.2'},
            {'lidar_max_relevant_dist': '5.0'},
            {'lidar_angle_range': '60.0'},
            {'lidar_min_safe_distance': '0.5'},
            {'use_camera_obstacles': 'false'},
            {'use_lane_following': 'false'},
            {'fusion_rate': '10.0'},
        ],
        output='screen'
    )

    # ── 2. Lane Follower (speed scaling only) ──────────────────────
    # Lane following disabled → only applies speed_factor scaling
    lane_follower = Node(
        package='ackerman_pkg',
        executable='lane_follower_node.py',
        name='lane_follower_node',
        parameters=[
            {'enable_lane_following': 'false'},   # No steering correction
            {'lane_offset_gain': '0.0'},
            {'max_steering_angle': '0.5'},
            {'min_speed_factor': '0.05'},
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav2'),           # IN: Nav2 output
            ('/cmd_vel_lane_corrected', '/cmd_vel'),  # OUT: to obstacle manager
        ],
        output='screen'
    )

    # ── 3. Obstacle Manager (5s stop + replan) ────────────────────
    obstacle_manager = Node(
        package='ackerman_pkg',
        executable='obstacle_manager_node.py',
        name='obstacle_manager_node',
        parameters=[
            {'critical_range_m': critical_range},
            {'stop_duration_sec': '5.0'},
            {'movement_threshold_m': '0.15'},
            {'check_angle_deg': '60.0'},
            {'pass_rate_hz': '20.0'},
        ],
        output='screen'
    )

    # ── 4. Arduino Bridge ─────────────────────────────────────────
    arduino_bridge = Node(
        package='ackerman_pkg',
        executable='arduino_bridge_node.py',
        name='arduino_bridge_node',
        parameters=[
            {'serial_port': arduino_port},
            {'baud_rate': '115200'},
            {'max_speed_mps': '1.0'},
            {'max_steer_rad': '0.6'},
            {'steer_center': '90'},
            {'steer_range': '45'},
            {'speed_max_pwm': '200'},
            {'cmd_timeout': '0.5'},
        ],
        output='screen'
    )

    return LaunchDescription([
        arduino_port_arg,
        critical_range_arg,
        sensor_fusion,
        lane_follower,
        obstacle_manager,
        arduino_bridge,
    ])
