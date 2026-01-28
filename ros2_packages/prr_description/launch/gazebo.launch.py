import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_name = 'prr_description'
    file_subpath = 'urdf/prr.urdf.xacro'

    # 1. Robot Modelini Isle
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    # 2. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 3. Yeni Gazebo'yu Baslat (Bos bir dunya ile)
    # -r: simulation calisir durumda baslar
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'),
                          'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 4. Robotu Gazebo'ya "Spawn" et (Yerlestir)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'my_robot',
                   '-z', '0.05'], # Yere gomulmemesi icin biraz yukaridan birakiyoruz
        output='screen'
    )

    # 5. Bridge (Opsiyonel ama gerekli olabilir - Clock bridge ornegi)
    # Gazebo saati ile ROS saatinin senkron olmasi icin
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge
    ])
