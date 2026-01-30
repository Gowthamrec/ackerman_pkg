import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    """
    Launch file for vision system using laptop camera only (no simulation)
    - Streams from /dev/video0 (laptop webcam)
    - Runs object detection
    - Runs lane detection
    """

    # 1. Camera stream from laptop (publishes to /camera/image_raw)
    camera_node = Node(
        package='image_tools',
        executable='cam2image',
        name='laptop_camera',
        parameters=[
            {'device_id': 0},
            {'output_topic': '/camera/image_raw'},
            {'frequency': 10.0}
        ],
        output='screen'
    )

    # 2. Object detection node
    object_detection_node = Node(
        package='ackerman_pkg',
        executable='object_detection_node.py',
        name='object_detection_node',
        parameters=[
            {'confidence_threshold': '0.6'},
            {'enable_visualization': 'true'}
        ],
        output='screen'
    )

    # 3. Lane detection node
    lane_detection_node = Node(
        package='ackerman_pkg',
        executable='lane_detection_node.py',
        name='lane_detection_node',
        parameters=[
            {'enable_visualization': 'true'}
        ],
        output='screen'
    )

    return LaunchDescription([
        camera_node,
        object_detection_node,
        lane_detection_node,
    ])
