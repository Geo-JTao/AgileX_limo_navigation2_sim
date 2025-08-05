import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    robot_name_in_model = 'limo'
    package_name = 'limo_description'
    urdf_name = "limo_four_diff.xacro"
    rviz_config_file_path = 'rviz/rviz.rviz'
    
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)
    gazebo_world_path = os.path.join(pkg_share, 'worlds/test1.world')

    # 解析XACRO
    robot_description = ParameterValue(
        Command(['xacro ', urdf_model_path]),
        value_type=str)

    # Gazebo启动（保持不变）
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', gazebo_world_path],
        output='screen')

    # 机器人状态发布器（改进版）
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen')

    # 实体生成（改进版）
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model, '-topic', 'robot_description'],
        output='screen')

    # 控制执行顺序
    delayed_spawn = RegisterEventHandler(
        OnProcessStart(
            target_action=start_robot_state_publisher_cmd,
            on_start=[spawn_entity_cmd]
        )
    )

    return LaunchDescription([
        start_gazebo_cmd,
        start_robot_state_publisher_cmd,
        delayed_spawn
    ])