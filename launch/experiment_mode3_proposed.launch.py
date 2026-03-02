#!/usr/bin/env python3
"""
Mode 3: Proposed (Nav2 + object detection + lane detection + risk-based fusion + lane follower)
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    object_detection = Node(
        package='ackerman_pkg',
        executable='object_detection_node.py',
        name='object_detection_node',
        parameters=[
            {'camera_topic': '/gazebo_camera/image_raw'},
            {'confidence_threshold': '0.5'},
            {'enable_visualization': 'false'},
        ],
        output='screen'
    )

    lane_detection = Node(
        package='ackerman_pkg',
        executable='lane_detection_node.py',
        name='lane_detection_node',
        parameters=[
            {'camera_topic': '/gazebo_camera/image_raw'},
            {'enable_visualization': 'false'},
        ],
        output='screen'
    )

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
            ('/cmd_vel', '/cmd_vel_nav'),
            ('/cmd_vel_lane_corrected', '/cmd_vel'),
        ],
        output='screen'
    )

    runner = Node(
        package='ackerman_pkg',
        executable='experiment_runner.py',
        name='experiment_runner',
        parameters=[
            {'mode': 'proposed'},
            {'num_runs': '20'},
            {'goal_x': '5.0'},
            {'goal_y': '3.0'},
            {'goal_yaw': '0.0'},
            {'start_x': '0.0'},
            {'start_y': '0.0'},
            {'start_yaw': '0.0'},
            {'timeout_sec': '120.0'},
            {'collision_threshold': '0.18'},
            {'goal_tolerance': '0.8'},
            {'settle_time': '3.0'},
            {'robot_name': 'my_bot'},
            {'vary_positions': 'true'},
        ],
        output='screen'
    )

    return LaunchDescription([
        object_detection,
        lane_detection,
        sensor_fusion,
        lane_follower,
        runner,
    ])
