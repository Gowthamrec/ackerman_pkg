import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('ackerman_pkg')

    # 1. Launch the Gazebo simulation and robot model
    gazebo_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'my_gz.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2. Launch the localization system
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'slam_map.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 3. Launch the navigation stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    return LaunchDescription([
        gazebo_sim_launch,
        localization_launch,
        nav2_launch
    ])