#!/usr/bin/env python3
"""
Autonomous Navigation — Complete System (LiDAR + Camera + Lane)
-----------------------------------------------------------------
Full autonomous navigation with all sensors active.
Optimal fusion weights (validated over 120 experiments):
  α=0.4 (LiDAR), β=0.4 (Camera), γ=0.2 (Lane)

DATA FLOW:
  /camera/image_raw (Camera)
      ├──► object_detection_node  → /obstacles_detected
      └──► lane_detection_node    → /lane_offset, /lane_status

  /scan (LiDAR)
      └──► sensor_fusion_node (α=0.4, β=0.4, γ=0.2)
                ├── reads /scan, /obstacles_detected, /lane_offset
                ├── → /risk_score [0-1]
                ├── → /risk_level NORMAL/CAUTION/SLOW/EMERGENCY
                ├── → /speed_factor 1.0/0.6/0.35/0.1
                └── → /lane_offset_fused (0 if obstacle present)

  Nav2 → /cmd_vel_nav2
      └──► lane_follower_node (steering + speed correction)
                └── /cmd_vel
                      └──► obstacle_manager_node (5s stop + replan)
                                  └── /cmd_vel_safe
                                          └──► arduino_bridge_node
                                                    └── Serial → Arduino Mega
                                                              └── Motors + Steering

USAGE:
  Terminal 1 — LiDAR driver:
    ros2 launch ackerman_pkg lidar_hardware.launch.py serial_port:=/dev/ttyUSB0

  Terminal 2 — Localization:
    ros2 launch ackerman_pkg localization.launch.py   ← needs existing map
    OR
    ros2 launch ackerman_pkg slam.launch.py           ← building new map

  Terminal 3 — Nav2:
    ros2 launch ackerman_pkg nav2.launch.py

  Terminal 4 — THIS FILE (full autonomous):
    ros2 launch ackerman_pkg autonomous_complete.launch.py

  Terminal 5 — Visualization (optional):
    ros2 launch ackerman_pkg my_bot_rviz.launch.py

PARAMETERS (override at launch time):
  camera_topic:=/camera/image_raw  ← change if your camera uses different topic
  arduino_port:=/dev/ttyACM0       ← change to your Arduino port
  critical_range:=0.30             ← emergency stop distance (meters)

MONITOR:
  ros2 topic echo /risk_level        ← NORMAL/CAUTION/SLOW/EMERGENCY
  ros2 topic echo /obstacle_status   ← MONITORING/CRITICAL_STOP/CHECKING/...
  ros2 topic echo /arduino_status    ← Arduino connection + last command
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # ── Launch arguments ──
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera image topic'
    )
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

    camera_topic = LaunchConfiguration('camera_topic')
    arduino_port = LaunchConfiguration('arduino_port')
    critical_range = LaunchConfiguration('critical_range')

    # ── 1. Object Detection (YOLOv8n) ────────────────────────────
    # Reads: /camera/image_raw
    # Writes: /detections, /obstacles_detected, /object_detection/image
    object_detection = Node(
        package='ackerman_pkg',
        executable='object_detection_node.py',
        name='object_detection_node',
        parameters=[
            {'camera_topic': camera_topic},
            {'confidence_threshold': '0.5'},
            {'enable_visualization': 'true'},
        ],
        output='screen'
    )

    # ── 2. Lane Detection (HSV) ───────────────────────────────────
    # Reads: /camera/image_raw
    # Writes: /lane_status, /lane_offset, /lane_detection/image
    lane_detection = Node(
        package='ackerman_pkg',
        executable='lane_detection_node.py',
        name='lane_detection_node',
        parameters=[
            {'camera_topic': camera_topic},
            {'enable_visualization': 'true'},
            {'lower_hsv': [0, 0, 200]},    # White lane lower bound (HSV)
            {'upper_hsv': [180, 30, 255]}, # White lane upper bound (HSV)
        ],
        output='screen'
    )

    # ── 3. Sensor Fusion (Optimal weights α=0.4, β=0.4, γ=0.2) ──
    # Reads: /scan, /obstacles_detected, /lane_offset, /lane_status
    # Writes: /obstacles_fused, /lane_offset_fused, /risk_score,
    #         /risk_level, /speed_factor, /sensor_fusion_status
    sensor_fusion = Node(
        package='ackerman_pkg',
        executable='sensor_fusion_node.py',
        name='sensor_fusion_node',
        parameters=[
            {'alpha_lidar': '0.4'},         # LiDAR weight (optimal)
            {'beta_camera': '0.4'},         # Camera weight (optimal)
            {'gamma_lane': '0.2'},          # Lane weight (optimal)
            {'emergency_threshold': '0.8'}, # R >= 0.8 → EMERGENCY (10% speed)
            {'slow_threshold': '0.5'},      # R >= 0.5 → SLOW (35% speed)
            {'caution_threshold': '0.2'},   # R >= 0.2 → CAUTION (60% speed)
            {'lidar_max_relevant_dist': '5.0'},
            {'lidar_angle_range': '60.0'},  # Front ±30° sector
            {'lidar_min_safe_distance': '0.5'},
            {'use_camera_obstacles': 'true'},
            {'use_lane_following': 'true'},
            {'fusion_rate': '10.0'},
        ],
        output='screen'
    )

    # ── 4. Lane Follower (risk-gated steering + speed scaling) ────
    # Reads: /cmd_vel_nav2, /lane_offset_fused, /speed_factor, /risk_level
    # Writes: /cmd_vel
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
            ('/cmd_vel', '/cmd_vel_nav2'),           # IN: from Nav2
            ('/cmd_vel_lane_corrected', '/cmd_vel'),  # OUT: to obstacle manager
        ],
        output='screen'
    )

    # ── 5. Obstacle Manager (5s stop + dynamic/static check) ──────
    # Reads: /scan, /cmd_vel, /risk_level
    # Writes: /cmd_vel_safe, /obstacle_status, /replan_trigger
    #
    # LOGIC:
    #   obstacle < critical_range → STOP 5s → check scan
    #   if moved: dynamic → resume path
    #   if static: trigger Nav2 replan → new shortest path
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

    # ── 6. Arduino Bridge (Serial → Arduino Mega → Motors) ────────
    # Reads: /cmd_vel_safe
    # Sends: S<speed_pwm>,T<steer_pwm>\n over serial to Arduino
    arduino_bridge = Node(
        package='ackerman_pkg',
        executable='arduino_bridge_node.py',
        name='arduino_bridge_node',
        parameters=[
            {'serial_port': arduino_port},
            {'baud_rate': '115200'},
            {'max_speed_mps': '1.0'},       # Max robot speed (m/s)
            {'max_steer_rad': '0.6'},       # Max steering (rad)
            {'steer_center': '90'},          # Servo center (degrees)
            {'steer_range': '45'},           # Servo ± range (degrees)
            {'speed_max_pwm': '200'},        # Max motor PWM (0-255)
            {'cmd_timeout': '0.5'},          # Watchdog timeout (sec)
        ],
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        camera_topic_arg,
        arduino_port_arg,
        critical_range_arg,

        # Vision (camera-based)
        object_detection,
        lane_detection,

        # Risk fusion
        sensor_fusion,

        # Control chain
        lane_follower,       # Nav2 → risk-gated cmd
        obstacle_manager,    # Smart 5s stop + replan
        arduino_bridge,      # Final → Arduino → motors
    ])
