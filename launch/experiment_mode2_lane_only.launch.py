#!/usr/bin/env python3
"""
Mode 2: Lane-only (Nav2 + lane detection + lane follower; no risk fusion)
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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
            ('/cmd_vel', '/cmd_vel_nav'),            # input from Nav2 (set in nav2 launch remap)
            ('/cmd_vel_lane_corrected', '/cmd_vel'), # output to robot
            ('/lane_offset_fused', '/lane_offset'),  # use raw lane offset
        ],
        output='screen'
    )

    runner = Node(
        package='ackerman_pkg',
        executable='experiment_runner.py',
        name='experiment_runner',
        parameters=[
            {'mode': 'lane_only'},
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
        lane_detection,
        lane_follower,
        runner,
    ])
