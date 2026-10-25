#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
localization.launch.py — 启动定位:
  1. ekf_node   : 融合 /odom + /imu/data -> /odometry/filtered + TF odom->base_footprint
  2. 静态TF     : include demo_cpp_tf 的 static_tf.launch.py(发 base_footprint->base_link->laser_link)

TF 链(每段唯一发布者):
  odom --(EKF)--> base_footprint --(demo_cpp_tf 静态)--> base_link --> laser_link

用法:
  ros2 launch robot_bringup localization.launch.py
  ros2 launch robot_bringup localization.launch.py publish_static_tf:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    base_link_z = LaunchConfiguration('base_link_z')
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_static_tf = LaunchConfiguration('publish_static_tf')
    params_file = LaunchConfiguration('params_file')

    declared = [
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='是否使用仿真时钟 (真机=false)'),
        DeclareLaunchArgument('base_link_z', default_value='0.06',
                              description='base_footprint -> base_link 高度差(m)'),
        DeclareLaunchArgument('publish_static_tf', default_value='true',
                              description='是否发布静态TF (base_footprint->base_link->laser_link)'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('robot_bringup'), 'config', 'ekf.yaml'
            ]),
            description='EKF 参数文件'),
    ]

    # ---- EKF: 融合 odom + imu, 发布 TF odom->base_footprint ----
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ---- 静态 TF: 交给 demo_cpp_tf 统一发布 ----
    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('demo_cpp_tf'), 'launch', 'static_tf.launch.py'
            ])
        ]),
        launch_arguments={'base_link_z': base_link_z,
                          'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(publish_static_tf),
    )

    return LaunchDescription(declared + [ekf_node, static_tf])
