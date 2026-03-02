#!/usr/bin/env python3
"""
Mode 1: Baseline (Nav2 only)
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    runner = Node(
        package='ackerman_pkg',
        executable='experiment_runner.py',
        name='experiment_runner',
        parameters=[
            {'mode': 'baseline'},
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

    return LaunchDescription([runner])
