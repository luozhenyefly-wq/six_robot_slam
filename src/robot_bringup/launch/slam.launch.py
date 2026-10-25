#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
slam.launch.py — 启动 slam_toolbox 建图 (在线异步)。
include 官方 slam_toolbox 的 online_async_launch.py, 只把参数换成本包的。

TF 链: map --(slam_toolbox)--> odom --(EKF)--> base_footprint --> base_link --> laser_link

用法:
  ros2 launch robot_bringup slam.launch.py
  ros2 launch robot_bringup slam.launch.py use_sim_time:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('slam_params_file')

    declared = [
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='是否使用仿真时钟'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('robot_bringup'), 'config', 'slam_toolbox.yaml'
            ]),
            description='slam_toolbox 参数文件'),
    ]

    # include 官方 online_async_launch.py, 覆盖 params_file
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'), 'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': params_file,
        }.items(),
    )

    return LaunchDescription(declared + [slam])
