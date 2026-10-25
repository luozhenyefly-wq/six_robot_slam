#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot.launch.py — 一键启动 (总入口)。

默认拉起: 传感器(uart_imu + 雷达) + 定位(EKF + 静态TF)
可选:     slam:=true 加建图,  nav:=true 加导航

用法:
  ros2 launch robot_bringup robot.launch.py                    # 传感器 + EKF
  ros2 launch robot_bringup robot.launch.py slam:=true         # + slam_toolbox
  ros2 launch robot_bringup robot.launch.py nav:=true map:=/path/map.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(filename, condition=None, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_bringup'), 'launch', filename
            ])
        ]),
        condition=condition,
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    do_slam = LaunchConfiguration('slam')
    do_nav = LaunchConfiguration('nav')
    map_yaml = LaunchConfiguration('map')
    base_link_z = LaunchConfiguration('base_link_z')

    declared = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('slam', default_value='false',
                              description='是否启动 slam_toolbox 建图'),
        DeclareLaunchArgument('nav', default_value='false',
                              description='是否启动 Nav2 导航'),
        DeclareLaunchArgument('map', default_value='',
                              description='Nav2 用的地图 yaml (nav:=true 且非 slam 时)'),
        DeclareLaunchArgument('base_link_z', default_value='0.06',
                              description='base_footprint -> base_link 高度差(m)'),
    ]

    sensors = _include('sensors.launch.py',
                       arguments={'use_sim_time': use_sim_time})

    localization = _include('localization.launch.py',
                            arguments={'base_link_z': base_link_z,
                                       'use_sim_time': use_sim_time})

    slam = _include('slam.launch.py',
                    condition=IfCondition(do_slam),
                    arguments={'use_sim_time': use_sim_time})

    nav = _include('nav2.launch.py',
                   condition=IfCondition(do_nav),
                   arguments={'use_sim_time': use_sim_time, 'map': map_yaml})

    return LaunchDescription(declared + [sensors, localization, slam, nav])
