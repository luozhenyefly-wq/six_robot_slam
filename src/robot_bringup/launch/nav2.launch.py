#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
nav2.launch.py — 启动 Nav2 导航 (真机, 无仿真, 无地图服务器)。
include 官方 nav2_bringup bringup_launch.py, 只换本包参数。

前提:
  - 已有地图 (map) 或改用 slam:=True 边建边导
  - TF 链完整: map->odom->base_footprint->base_link->laser_link

用法:
  ros2 launch robot_bringup nav2.launch.py map:=/path/to/map.yaml
  ros2 launch robot_bringup nav2.launch.py slam:=True      # 用 slam_toolbox 当定位
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    slam = LaunchConfiguration('slam')

    declared = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('slam', default_value='False',
                              description='True=用 slam_toolbox 当定位(无需地图文件)'),
        DeclareLaunchArgument('map', default_value='',
                              description='地图 yaml 路径(不使用 slam 时必填)'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('robot_bringup'), 'config', 'nav2_params.yaml'
            ]),
            description='Nav2 参数文件'),
    ]

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': slam,
            'map': map_yaml,
            'params_file': params_file,
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription(declared + [nav2])
