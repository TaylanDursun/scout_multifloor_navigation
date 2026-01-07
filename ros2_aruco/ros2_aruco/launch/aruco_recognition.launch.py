import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Aruco Node Tanımlaması
    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        output='screen',
        parameters=[{
            'marker_size': 0.2,                      # Gazebo'daki marker boyutu (20cm)
            'aruco_dictionary_id': 'DICT_5X5_250',   # Kullandığımız marker sözlüğü
            'image_topic': '/zed_camera/image_raw',  # ZED Kameranın görüntü topic'i
            'camera_info_topic': '/zed_camera/camera_info', # ZED Kameranın kalibrasyon topic'i
            'camera_frame': 'camera_optical_frame'   # Kameranın referans çerçevesi
        }]
    )

    return LaunchDescription([
        aruco_node
    ])
