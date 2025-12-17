import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取包的安装路径
    pkg_share = get_package_share_directory('mimo_adrc')

    # 2. 指定 Rviz 配置文件的路径 (虽然现在还没存，但我们先占个位)
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'view.rviz')

    # 3. 定义仿真节点
    mimo_node = Node(
        package='mimo_adrc',
        executable='mimo_node',
        name='voliro_sim_node',
        output='screen'
    )

    # 4. 定义 Rviz2 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # 加载配置文件 (如果文件不存在，它会加载默认配置，没关系)
        arguments=['-d', rviz_config_path],
        # 【关键】WSL 防黑屏补丁
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        output='screen'
    )

    return LaunchDescription([
        mimo_node,
        rviz_node
    ])
