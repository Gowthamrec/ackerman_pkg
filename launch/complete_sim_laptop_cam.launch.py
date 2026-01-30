import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Complete simulation with laptop camera:
    - Gazebo simulation (robot, lidar)
    - RViz visualization
    - Laptop camera streaming (REAL WORLD - not simulation)
    - Object detection + lane detection (from laptop camera - NOT Gazebo!)
    """

    pkg_share_dir = get_package_share_directory('ackerman_pkg')

    # 1. Robot State Publisher (from my_bot_rviz.launch.py)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share_dir, 'launch', 'my_bot_rviz.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2. Gazebo simulation launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        )
    )

    # 3. Spawn robot entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_bot',
                   '-z', '0.25'],
        output='screen'
    )

    # 4. Laptop camera (REAL WORLD camera - publishes to /camera/image_raw)
    laptop_camera = Node(
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

    # 5. Object detection node (processes REAL laptop camera from /image)
    object_detection_node = Node(
        package='ackerman_pkg',
        executable='object_detection_node.py',
        name='object_detection_node',
        parameters=[
            {'camera_topic': '/image'},
            {'confidence_threshold': '0.6'},
            {'enable_visualization': 'true'}
        ],
        output='screen'
    )

    # 6. Lane detection node (processes REAL laptop camera from /image)
    lane_detection_node = Node(
        package='ackerman_pkg',
        executable='lane_detection_node.py',
        name='lane_detection_node',
        parameters=[
            {'camera_topic': '/image'},
            {'enable_visualization': 'true'}
        ],
        output='screen'
    )
    
    # 7. Visual odometry node (processes REAL laptop camera from /image)
    visual_odometry_node = Node(
        package='ackerman_pkg',
        executable='visual_odometry_node.py',
        name='visual_odometry_node',
        parameters=[
            {'camera_topic': '/image'},
            {'publish_tf': 'false'}
        ],
        output='screen'
    )

    # 8. Sensor Fusion Node (LiDAR HIGH PRIORITY + Camera MEDIUM PRIORITY)
    sensor_fusion_node = Node(
        package='ackerman_pkg',
        executable='sensor_fusion_node.py',
        name='sensor_fusion_node',
        parameters=[
            {'lidar_range_threshold': '0.5'},  # 0.5m - HIGH PRIORITY detection
            {'lidar_angle_range': '60.0'},     # Front 60 degrees
            {'use_camera_obstacles': 'true'},
            {'use_lane_following': 'true'},
            {'lidar_priority': 'true'}         # LiDAR overrides camera
        ],
        output='screen'
    )

    # 9. Lane follower node (uses FUSED lane offset - safe when no obstacles)
    lane_follower_node = Node(
        package='ackerman_pkg',
        executable='lane_follower_node.py',
        name='lane_follower_node',
        parameters=[
            {'enable_lane_following': 'true'},
            {'lane_offset_gain': '0.5'},
            {'max_steering_angle': '0.5'}
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),
            ('/lane_offset', '/lane_offset_fused'),  # Use FUSED lane offset (0 if obstacle)
            ('/cmd_vel_lane_corrected', '/cmd_vel')
        ],
        output='screen'
    )

    return LaunchDescription([
        # Start laptop camera FIRST (before Gazebo)
        laptop_camera,
        
        # Gazebo simulation components (NO vision nodes from Gazebo)
        rsp,
        gazebo,
        spawn_entity,
        
        # Vision nodes (process real laptop camera)
        object_detection_node,
        lane_detection_node,
        visual_odometry_node,
        
        # Sensor Fusion (LiDAR HIGH PRIORITY)
        sensor_fusion_node,
        
        # Lane following controller (uses fused data)
        lane_follower_node,
    ])
