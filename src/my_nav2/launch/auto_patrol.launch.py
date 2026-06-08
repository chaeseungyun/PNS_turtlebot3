import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 인자
    use_rviz = LaunchConfiguration('use_rviz', default='false')
    map_file = LaunchConfiguration('map', 
        default=os.path.join(
            get_package_share_directory('my_nav2'),
            'map', 'office_clean.yaml'))
    
    # 1. 기존 Nav2 launch 포함 (단, RViz 옵션 전달)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('my_nav2'),
                         'launch', 'my_navigation2.launch.py')
        ]),
        launch_arguments={
            'map': map_file,
        }.items(),
    )
    
    # 2. Auto Patrol 노드
    auto_patrol = Node(
        package='my_nav2',
        executable='auto_patrol_node.py',
        name='auto_patrol',
        output='screen',
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Use RViz or not'),
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Map file path'),
        nav2_launch,
        auto_patrol,
    ])
