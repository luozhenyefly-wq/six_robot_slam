#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sensors.launch.py — 启动传感器:
  1. uart_imu    : 串口帧(yaw/vyaw/odom) -> /imu/data + /odom
  2. lslidar     : 镭神 N10-P 雷达 -> /scan

用法:
  ros2 launch robot_bringup sensors.launch.py
  ros2 launch robot_bringup sensors.launch.py imu_port:=/dev/ttyS1 lidar_port:=/dev/ttyACM0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---- 参数 ----
    use_sim_time = LaunchConfiguration('use_sim_time')
    imu_port = LaunchConfiguration('imu_port')
    imu_baud = LaunchConfiguration('imu_baudrate')
    lidar_launch = LaunchConfiguration('lidar_launch')

    declared = [
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='是否使用仿真时钟 (真机=false)'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyS1',
                              description='串口陀螺仪/里程计端口'),
        DeclareLaunchArgument('imu_baudrate', default_value='115200',
                              description='串口波特率'),
        DeclareLaunchArgument('lidar_launch', default_value='lslidar_n10p_bringup.launch.py',
                              description='雷达 launch 文件名'),
    ]

    # ---- uart_imu: 发 /imu/data + /odom (不发 TF, TF 交 EKF) ----
    uart_imu_node = Node(
        package='uart_imu',
        executable='uart_imu_node',
        name='uart_imu',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'port': imu_port,
            'baudrate': imu_baud,
            'unit': 'deg',          # 原始角度单位
            'odom_unit': 'cm',      # 原始里程单位(厘米)
            'frame_id': 'imu_link',
            'enable_odom': True,
            'publish_tf': False,    # ☆ TF 交给 EKF 发, 避免撞车
            'debug': False,
        }],
    )

    # ---- 雷达 (include 官方/驱动包自带 launch) ----
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lslidar_driver'), 'launch', lidar_launch
            ])
        ])
    )

    return LaunchDescription(declared + [uart_imu_node, lidar])
