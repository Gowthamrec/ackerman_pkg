#!/usr/bin/env python3
"""
hw_camera.launch.py
-------------------
Launches USB webcam driver → publishes /image_raw.
Then republishes as /image so all vision nodes use it.

Publishes:
  /camera/image_raw  (raw from driver)
  /image             (unified topic for all vision nodes)

Standalone test:
  ros2 launch ackerman_pkg hw_camera.launch.py
  ros2 topic echo /image --no-arr
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Video device for USB webcam'
    )
    width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='Camera image width'
    )
    height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='Camera image height'
    )
    fps_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30',
        description='Camera framerate'
    )

    # USB webcam driver node
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('camera_device'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'framerate': LaunchConfiguration('framerate'),
            'camera_frame_id': 'camera_link',
            'pixel_format': 'yuyv',
            'camera_name': 'usb_cam',
            'io_method': 'mmap',
        }],
        remappings=[
            ('/usb_cam/image_raw', '/camera/image_raw'),
        ]
    )

    # Relay: /camera/image_raw → /image  (unified topic for all vision nodes)
    image_relay = Node(
        package='topic_tools',
        executable='relay',
        name='camera_image_relay',
        output='screen',
        arguments=['/camera/image_raw', '/image'],
    )

    return LaunchDescription([
        device_arg,
        width_arg,
        height_arg,
        fps_arg,
        usb_cam_node,
        image_relay,
    ])
