# keepout_filter.launch.py — Nav2 KeepoutFilter 서버 2종 + 라이프사이클 매니저.
#
# 띄우는 노드:
#   filter_mask_server        : keepout 마스크(.yaml)를 OccupancyGrid로 퍼블리시
#   costmap_filter_info_server: CostmapFilterInfo(type=keepout) 퍼블리시
#   lifecycle_manager_costmap_filters : 위 둘을 configure/activate
#
# 사용(보통 my_navigation2.launch.py가 자동 include):
#   ros2 launch my_nav2 keepout_filter.launch.py
#   인자: mask:=<keepout_mask.yaml 절대경로>  params_file:=<waffle_pi.yaml>
#
# costmap 쪽 plugin 설정(keepout_filter)과 filter_info_topic/mask_topic 은
# param/humble/waffle_pi.yaml 안에 함께 정의돼 있다.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi')


def generate_launch_description():
    pkg = get_package_share_directory('my_nav2')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    mask_yaml = LaunchConfiguration(
        'mask', default=os.path.join(pkg, 'map', 'keepout_mask.yaml'))
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg, 'param', 'humble', TURTLEBOT3_MODEL + '.yaml'))

    lifecycle_nodes = ['filter_mask_server', 'costmap_filter_info_server']

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('mask', default_value=mask_yaml,
                              description='keepout 마스크 yaml 절대경로'),
        DeclareLaunchArgument('params_file', default_value=params_file,
                              description='필터 서버 파라미터(waffle_pi.yaml 재사용)'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time,
                                      'yaml_filename': mask_yaml}]),

        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}]),
    ])
