from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取地图文件的路径（假设地图文件在包的share目录下）
    package_name = 'fishbot_cartographer' 
    map_file_path = os.path.join(
        get_package_share_directory(package_name),
        'map',
        'fishbot_map.yaml'
    )
    # 静态 TF 发布（map → odom）
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # 创建map_server节点（作为生命周期节点）
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{'yaml_filename': map_file_path,
                     'publish_period': 1.0  # 关键：设置发布频率（秒）
                }]
    )

    # 创建rviz2节点
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # 自动配置和激活map_server的生命周期
    configure_map_server = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
        output='screen'
    )

    activate_map_server = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
        output='screen'
    )

    return LaunchDescription([
        map_server_node,
        rviz2_node,
        static_tf_node,
        # 延迟执行生命周期状态转换
        ExecuteProcess(
            cmd=['sleep', '2'],  # 等待map_server启动
            output='screen'
        ),
        configure_map_server,
        ExecuteProcess(
            cmd=['sleep', '1'],  # 等待配置完成
            output='screen'
        ),
        activate_map_server
    ])