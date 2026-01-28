import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Paket ismini ve dosya yollarini tanimliyoruz
    pkg_name = 'prr_description'
    file_subpath = 'urdf/prr.urdf.xacro'
    
    # Xacro dosyasinin tam yolunu buluyoruz
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    
    # Robot State Publisher Node'u
    # Bu node, xacro dosyasini okur ve robotun TF (transform) agacini yayinlar
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', xacro_file])}]
    )

    # Joint State Publisher GUI Node'u
    # Bu kucuk pencere sayesinde eklemleri elle (slider ile) hareket ettirecegiz
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # RViz Node'u
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
