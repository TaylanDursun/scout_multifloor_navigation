import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Robotun hangi katta nerede doğacağını belirleyen fonksiyon
def launch_setup(context, *args, **kwargs):
    # Terminalden girilen 'floor' argümanını oku
    floor_value = LaunchConfiguration('floor').perform(context)
    
    # --- KOORDİNAT AYARLARI ---
    # Floor 1 (Zemin Kat) için güvenli başlangıç noktası (Lobi)
    if floor_value == '1':
        spawn_x = '1.90'
        spawn_y = '11.10'
        spawn_z = '0.25' # Zemin kat yüksekliği
        print(f"--- FLOOR 1 SEÇİLDİ: Robot (1.90, 11.10) noktasına ışınlanıyor ---")
        
    # Floor 2 (Üst Kat) için
    elif floor_value == '2':
        spawn_x = '-0.3'
        spawn_y = '10.95'
        spawn_z = '3.2' # Üst kat yüksekliği
        print(f"--- FLOOR 2 SEÇİLDİ: Robot (-0.3, 10.95) noktasına ışınlanıyor ---")
        
    else:
        print(f"--- HATA: Geçersiz Kat! Varsayılan olarak Floor 2 seçildi ---")
        spawn_x = '-0.3'
        spawn_y = '10.95'
        spawn_z = '3.2'

    # Robotu Spawn Eden Node
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'scout_v2',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
        ],
        output='screen'
    )
    
    return [spawn_robot]

def generate_launch_description():
    scout_gazebo_sim_path = get_package_share_directory('scout_gazebo_sim')
    scout_description_path = get_package_share_directory('scout_description')
    
    # Dünya Dosyası Yolu
    world_file_name = 'hospital_two_floors_markes.world'
    world_path = os.path.join(scout_gazebo_sim_path, 'worlds', world_file_name)

    # Argüman Tanımları
    floor_arg = DeclareLaunchArgument(
        'floor',
        default_value='1',
        description='Secilecek Kat: "1" veya "2"'
    )

    # Gazebo Launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # Robot Description Launch
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scout_description_path, 'launch', 'display_scout_v2.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    

    return LaunchDescription([
        floor_arg,
        gazebo_launch,
        description_launch,
        OpaqueFunction(function=launch_setup)
    ])
