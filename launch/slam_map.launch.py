import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams


def generate_launch_description():

    # -----------------------------
    # Launch configurations
    # -----------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    default_params_file = os.path.join(
        get_package_share_directory('ackerman_pkg'),
        'config',
        'slam_map.yaml'
    )

    # -----------------------------
    # Declare launch arguments
    # -----------------------------
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',   # ✅ PERMANENT FIX
        description='Use simulation (Gazebo) clock'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file for slam_toolbox'
    )

    # -----------------------------
    # Check if provided params file contains slam_toolbox params
    # -----------------------------
    has_node_params = HasNodeParams(
        source_file=params_file,
        node_name='slam_toolbox'
    )

    actual_params_file = PythonExpression([
        '"', params_file, '" if ', has_node_params,
        ' else "', default_params_file, '"'
    ])

    log_param_change = LogInfo(
        msg=[
            'Provided params_file ',
            params_file,
            ' does not contain slam_toolbox parameters. Using default: ',
            default_params_file
        ],
        condition=UnlessCondition(has_node_params)
    )

    # -----------------------------
    # SLAM Toolbox node
    # -----------------------------
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            actual_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # -----------------------------
    # Launch description
    # -----------------------------
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params_file)
    ld.add_action(log_param_change)
    ld.add_action(slam_toolbox_node)

    return ld
