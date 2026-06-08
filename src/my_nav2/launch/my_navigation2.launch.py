# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
ROS_DISTRO = os.environ.get('ROS_DISTRO')


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('my_nav2'),
            'map',
            'map.yaml'))

    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    if ROS_DISTRO == 'humble':
        param_dir = LaunchConfiguration(
            'params_file',
            default=os.path.join(
                get_package_share_directory('my_nav2'),
                'param',
                ROS_DISTRO,
                param_file_name))
    else:
        param_dir = LaunchConfiguration(
            'params_file',
            default=os.path.join(
                get_package_share_directory('my_nav2'),
                'param',
                param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    my_nav2_launch_dir = os.path.join(get_package_share_directory('my_nav2'), 'launch')
    use_keepout = LaunchConfiguration('use_keepout', default='true')
    use_scan_filter = LaunchConfiguration('use_scan_filter', default='true')
    scan_mask_lo = LaunchConfiguration('scan_mask_lo', default='64.0')
    scan_mask_hi = LaunchConfiguration('scan_mask_hi', default='109.0')

    rviz_config_dir = os.path.join(
        get_package_share_directory('my_nav2'),
        'rviz',
        'tb3_navigation2.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'use_keepout',
            default_value='true',
            description='라이다가 못 보는 책상/의자 keepout 필터를 띄울지'),

        DeclareLaunchArgument(
            'use_scan_filter',
            default_value='true',
            description='좌측 다이나믹셀 자기차폐 띠를 마스킹하는 scan_arm_mask 노드를 띄울지. '
                        'param이 /scan_filtered를 구독하므로 보통 켜둔다(끄려면 raw /scan 쓰는 별도 param 필요)'),
        DeclareLaunchArgument(
            'scan_mask_lo', default_value='64.0',
            description='좌측 자기차폐 띠 하한(deg). band-pass: lo<=각도<=hi 를 마스킹'),
        DeclareLaunchArgument(
            'scan_mask_hi', default_value='109.0',
            description='좌측 자기차폐 띠 상한(deg). 실측 사각 66-107(2026-06-08)+여유. '
                        '마스킹 끄려면 lo>hi (예: lo=361 hi=0) → passthrough'),

        # 좌측 다이나믹셀 자기차폐 띠 마스킹: /scan -> /scan_filtered (param이 이 토픽을 구독)
        Node(
            package='my_nav2',
            executable='scan_arm_mask.py',
            name='scan_arm_mask',
            output='screen',
            condition=IfCondition(use_scan_filter),
            parameters=[{
                'use_sim_time': use_sim_time,
                'mask_lo': ParameterValue(scan_mask_lo, value_type=float),
                'mask_hi': ParameterValue(scan_mask_hi, value_type=float),
            }]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([my_nav2_launch_dir, '/keepout_filter.launch.py']),
            condition=IfCondition(use_keepout),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
