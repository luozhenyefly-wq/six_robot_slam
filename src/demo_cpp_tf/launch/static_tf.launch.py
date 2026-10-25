#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
static_tf.launch.py — 发布机器人所有静态 TF (base_footprint / base_link / laser_link)。

发布两段静态变换:
  base_footprint -> base_link   机身高度 (默认 z=0.06m)
  base_link      -> laser_link  雷达安装位 (默认 前3cm, 高9.6cm, yaw=+0.257262)

TF 链(每段唯一发布者):
  odom --(EKF)--> base_footprint --(本文件)--> base_link --(本文件)--> laser_link

用法:
  ros2 launch demo_cpp_tf static_tf.launch.py
  ros2 launch demo_cpp_tf static_tf.launch.py base_link_z:=0.06
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    base_link_z = LaunchConfiguration('base_link_z')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declared = [
        DeclareLaunchArgument('base_link_z', default_value='0.06',
                              description='base_footprint -> base_link 高度差(m)'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='是否使用仿真时钟 (真机=false)'),
    ]

    # 一次发布两段静态 TF
    #   parents / childs / xyz_rpy 一一对应; xyz_rpy 每段 6 个数 [x,y,z,roll,pitch,yaw]
    static_tf = Node(
        package='demo_cpp_tf',
        executable='static_tf_broadcaster',
        name='static_tf_broad',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'parents': ['base_footprint', 'base_link', 'base_link'],
            'childs': ['base_link', 'laser_link', 'imu_link'],
            'xyz_rpy': [
                0.0, 0.0, base_link_z, 0.0, 0.0, 0.0,      # base_footprint->base_link
                # base_link->laser_link: 雷达零位相对车体 +x 逆时针 120.26° = 2.098933 rad（90-14.74+45）
                #   z = 0.137 m（雷达高度实测）
                0.03, 0.0, 0.137, 0.0, 0.0, 2.098933,       # base_link->laser_link
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,         # base_link->imu_link (IMU 与车体同向同位置, 让 EKF 能取到变换)
            ],
        }],
    )

    return LaunchDescription(declared + [static_tf])
