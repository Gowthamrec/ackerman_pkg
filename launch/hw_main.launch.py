#!/usr/bin/env python3
"""
hw_main.launch.py
=================
MASTER LAUNCH FILE for full real-world autonomous operation.

Includes ALL components:
  1. hw_robot_state   → URDF + TF tree (base_link, laser, camera_link)
  2. hw_lidar         → RPLidar driver   → /scan
  3. hw_camera        → USB webcam       → /image
  4. hw_localization  → AMCL on saved map → /amcl_pose
  5. hw_nav2          → Nav2 stack       → path planning + cmd_vel_nav
  6. hw_vision        → YOLO + lane + fusion + follower → /cmd_vel (motors)

Usage:
  # Full autonomous navigation on saved map:
  ros2 launch ackerman_pkg hw_main.launch.py \
    map:=/home/ros2/car_project_ws/src/ackerman_pkg/map/my_map.yaml

  # With custom camera/lidar ports:
  ros2 launch ackerman_pkg hw_main.launch.py \
    map:=/path/to/my_map.yaml \
    serial_port:=/dev/ttyUSB0 \
    camera_device:=/dev/video0

TOPIC FLOW:
  RPLidar → /scan
  USB Cam → /camera/image_raw → /image
  AMCL    → /amcl_pose (localization on map)
  Nav2    → /cmd_vel_nav (path planning)
  Vision  → /obstacles_fused, /speed_factor
  Follower→ /cmd_vel (FINAL → to robot motors/Arduino)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('ackerman_pkg')
    launch_dir = os.path.join(pkg_dir, 'launch')
    default_map = os.path.join(pkg_dir, 'map', 'my_map.yaml')

    # ── Launch Arguments ─────────────────────────────────────────────────────
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to saved map .yaml file'
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='USB port for RPLidar'
    )
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Video device for USB webcam'
    )

    # ── 1. Robot State Publisher + TF Tree ───────────────────────────────────
    robot_state = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'hw_robot_state.launch.py')
        )
    )

    # ── 2. LiDAR Driver ──────────────────────────────────────────────────────
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'hw_lidar.launch.py')
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port'),
        }.items()
    )

    # ── 3. Camera Driver ─────────────────────────────────────────────────────
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'hw_camera.launch.py')
        ),
        launch_arguments={
            'camera_device': LaunchConfiguration('camera_device'),
        }.items()
    )

    # ── 4. Localization (AMCL on saved map) ──────────────────────────────────
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'hw_localization.launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
        }.items()
    )

    # ── 5. Nav2 Stack ─────────────────────────────────────────────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'hw_nav2.launch.py')
        )
    )

    # ── 6. Vision + Fusion + Lane Follower ────────────────────────────────────
    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'hw_vision.launch.py')
        )
    )

    banner = LogInfo(msg=[
        '\n',
        '=======================================================\n',
        '   REAL HARDWARE AUTONOMOUS SYSTEM STARTING\n',
        '=======================================================\n',
        '  LiDAR port   : ', LaunchConfiguration('serial_port'), '\n',
        '  Camera device: ', LaunchConfiguration('camera_device'), '\n',
        '  Map file     : ', LaunchConfiguration('map'), '\n',
        '-------------------------------------------------------\n',
        '  Set initial pose in RViz: "2D Pose Estimate"\n',
        '  Set nav goal  in RViz: "2D Nav Goal"\n',
        '=======================================================\n',
    ])

    return LaunchDescription([
        map_arg,
        serial_port_arg,
        camera_device_arg,
        banner,
        robot_state,   # Step 1: TF tree
        lidar,         # Step 2: /scan
        camera,        # Step 3: /image
        localization,  # Step 4: AMCL
        nav2,          # Step 5: path planning
        vision,        # Step 6: vision + stop + steer
    ])
