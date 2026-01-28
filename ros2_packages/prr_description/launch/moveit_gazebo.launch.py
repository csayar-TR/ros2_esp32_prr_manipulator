import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    
    # --- 0. ORTAK PARAMETRELER ---
    # En kritik kisim burasi: Tum node'lar sim zamanini kullanmali!
    sim_mode = {'use_sim_time': True}

    # --- 1. DOSYA YOLLARINI AYARLAMA ---
    description_pkg_share = get_package_share_directory('prr_description')
    xacro_file = os.path.join(description_pkg_share, 'urdf', 'prr.urdf.xacro')

    robot_description_content = Command(['xacro ', xacro_file])
    robot_description_param = {'robot_description': robot_description_content}

    # MoveIt Config
    moveit_config = (
        MoveItConfigsBuilder("my_robot", package_name="prr_moveit_config")
        .robot_description(file_path=xacro_file)
        .to_moveit_configs()
    )

    # --- 2. ROBOT STATE PUBLISHER ---
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        # Hem robot tanimini hem de sim_time parametresini veriyoruz
        parameters=[robot_description_param, sim_mode]
    )

    # --- 3. GAZEBO SIMULASYONU ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'),
                          'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # --- 4. ROBOTU SPAWN ETME ---
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'my_robot',
                   '-z', '0.05'],
        output='screen'
    )

    # --- 5. CONTROLLERLARI BASLATMA ---
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen",
    )

    delayed_controller_manager = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner, joint_trajectory_controller_spawner],
        )
    )

    # --- 6. MOVEIT (MOVE_GROUP) ---
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        # Sim mode'u buraya da ekliyoruz
        parameters=[moveit_config.to_dict(), sim_mode],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # --- 7. RVIZ ---
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("prr_moveit_config"), "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            sim_mode  # Sim mode buraya da eklendi
        ],
    )

    # --- 8. BRIDGE ---
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
        delayed_controller_manager,
        move_group_node,
        rviz_node,
        bridge
    ])
