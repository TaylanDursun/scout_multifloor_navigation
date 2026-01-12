from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_launch_dir = get_package_share_directory('nav2_bringup')
    
    # Varsayılan harita yolu (floor1)
    map_yaml_file = '/home/diakamos/ros2_ws/src/team4_multifloor/team4_maps/maps/floor1.yaml'

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml_file,
                'use_sim_time': 'true',
                'params_file': os.path.join(nav2_launch_dir, 'params', 'nav2_params.yaml')
            }.items()
        )
    ])
