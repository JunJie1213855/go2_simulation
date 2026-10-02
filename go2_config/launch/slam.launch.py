# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# SLAM (建图) + Nav2 导航 一体化启动文件
#
# 用法（配合 Gazebo 仿真）:
#   ros2 launch go2_config gazebo.launch.py
#   ros2 launch go2_config slam.launch.py
#
# 说明:
#   - slam_toolbox 在线异步建图（同时提供 map->odom 变换，即定位）
#   - nav2 导航栈（bt_navigator / planner / controller / costmap）
#   - rviz 可视化（默认开启）
#
# 建完图后保存地图:
#   ros2 run nav2_map_server map_saver_cli -f ~/map
# 之后可改用 navigate.launch.py（AMCL 定位 + 导航，需要已保存的地图）。

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    this_package = FindPackageShare('go2_config')

    default_slam_params_file = PathJoinSubstitution(
        [this_package, 'config/autonomy', 'slam.yaml']
    )

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
    )

    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('champ_navigation'), 'rviz', 'slam.rviz']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use the Gazebo /clock (set to false for real hardware)'
        ),

        DeclareLaunchArgument(
            name='slam_params_file',
            default_value=default_slam_params_file,
            description='slam_toolbox params file'
        ),

        DeclareLaunchArgument(
            name='rviz',
            default_value='true',
            description='Run rviz'
        ),

        # Nav2 导航栈（SLAM 模式下不需要 map_server / AMCL，slam_toolbox 负责 map->odom）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items()
        ),

        # slam_toolbox 在线异步建图
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': LaunchConfiguration('slam_params_file')
            }.items()
        ),

        # rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])
