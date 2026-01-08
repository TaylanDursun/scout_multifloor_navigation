# Eski launch dosyası (bu dosyadaspawn olacak kat belirlenemiyor)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Paket Yollarını Tanımla
    scout_gazebo_sim_path = get_package_share_directory('scout_gazebo_sim')
    scout_description_path = get_package_share_directory('scout_description')
    
    # 2. Dünya Dosyası Yolu (Hospital World)
    world_file_name = 'hospital_two_floors.world'
    world_path = os.path.join(scout_gazebo_sim_path, 'worlds', world_file_name)

    # 3. Gazebo Başlatma (world parametresi ile)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # 4. Robot Description (URDF/Xacro) Başlatma
    # HW2'de yaptığın sensörlü robot tanımını çağırıyoruz
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scout_description_path, 'launch', 'display_scout_v2.launch.py')
        )
    )

    # 5. Robotu Spawn Etme (Gazebo içine koyma)
    # Robotu biraz yukarıdan (z=0.5) bırakıyoruz ki yere saplanmasın
    spawn_robot = Node(
    	package='gazebo_ros',
    	executable='spawn_entity.py',
    	arguments=['-topic', 'robot_description', 
               	'-entity', 'scout_v2',
               	'-x', '-0.3',   
               	'-y', '10.95', 
               	'-z', '3.2'], 
    	output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        description_launch,
        spawn_robot
    ])
