# six_robot_slam

> 六足机器人（six_robot）的 **Linux 上位机 ROS 2 程序** —— 激光雷达 + 串口 IMU/里程计的
> SLAM 建图与定位框架，运行于 D-Robotics RDK X5（aarch64 / Ubuntu 22.04 / ROS 2 Humble）。

📦 下位机（STM32）固件与硬件工程：[luozhenyefly-wq/six_robot](https://github.com/luozhenyefly-wq/six_robot)

---

## 这是什么

本仓库是六足机器人系统的**上位机部分**，负责：

- 读取 **镭神 N10-P 激光雷达** → 发布 `/scan`
- 读取 **STM32 下位机串口帧**（yaw / 偏航角速度 / 里程增量）→ 发布 `/imu/data` 与 `/odom`
- 用 **robot_localization (EKF)** 融合 IMU 与轮/足式里程计 → 发布 `odom → base_footprint` TF
- 用 **slam_toolbox** 做 2D 激光 SLAM → 发布 `map → odom` TF 与 `/map` 栅格地图
- 预留 **Nav2** 导航接口（launch 已就位，参数待补）

完整 TF 树：

```
map ──(slam_toolbox)──> odom ──(EKF)──> base_footprint ──(static)──> base_link ──(static)──> laser_link
                                                                  └─(static)──> imu_link
```

每一段 TF 只有一个发布者，符合 REP-105。

---

## 仓库结构

```
robot_ws/
├── src/
│   ├── robot_bringup/        一键启动：sensors / localization / slam / nav2 组合
│   │   ├── launch/
│   │   │   ├── robot.launch.py          总入口
│   │   │   ├── sensors.launch.py        uart_imu + 雷达
│   │   │   ├── localization.launch.py   EKF + 静态 TF
│   │   │   ├── slam.launch.py           slam_toolbox
│   │   │   └── nav2.launch.py           Nav2（预留）
│   │   └── config/
│   │       ├── ekf.yaml                 robot_localization 参数
│   │       └── slam_toolbox.yaml        建图参数
│   │
│   ├── uart_imu/             C++ 串口节点：帧解析 → Imu + Odometry
│   ├── demo_cpp_tf/          静态 / 动态 TF 广播与监听示例
│   ├── lslidar_driver/       镭神雷达驱动（第三方）
│   ├── lslidar_msgs/         镭神雷达消息定义（第三方）
│   └── robot_navigation2/    地图与导航资源（导航未启用）
│
├── docs/
│   ├── 01_overview.md        项目说明：架构 / TF 树 / 话题 / 硬件 / 协议
│   ├── 02_environment.md     环境配置：ROS 2 Humble + slam_toolbox + rviz
│   ├── 03_build.md           编译说明
│   ├── 04_run_slam.md        建图运行流程
│   └── 05_troubleshooting.md 常见问题排查
│
├── LICENSE                   MIT（第三方包除外）
└── README.md
```

---

## 快速开始

```bash
# 1. 准备环境（详见 docs/02_environment.md）
sudo apt install ros-humble-desktop ros-humble-robot-localization \
                 ros-humble-slam-toolbox ros-humble-nav2-bringup

# 2. 编译
cd ~/robot_ws
colcon build --symlink-install

# 3. 加载环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 4. 建图
ros2 launch robot_bringup robot.launch.py slam:=true

# 5. 另一个终端看地图
rviz2 -d src/robot_bringup/config/slam.rviz
```

保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

---

## 硬件依赖

| 部件 | 型号 | 接口 | 说明 |
|---|---|---|---|
| 主控 | D-Robotics RDK X5 | — | aarch64, Ubuntu 22.04, ROS 2 Humble |
| 激光雷达 | 镭神 N10-P | USB (`/dev/ttyACM0`) | 10 m 级 2D 单线雷达，400000 波特率 |
| 下位机 | STM32 | UART (`/dev/ttyS1`) | 115200 8N1，自定义帧协议 |
| IMU | 集成于下位机 | 同上 | 六轴相对角度，上电即零点 |

串口帧协议（下位机 → 上位机）：

```
0xAA | yaw (int16, 大端) | vyaw (int16, 大端) | ... | 0xBB
```

- `yaw`：单位 **0.01°**，六轴相对角度，上电清零
- `vyaw`：单位 **0.01°/s**
- 里程：单位 **cm**

坐标系约定：**x 前、y 左、z 上**；ROS 角度**逆时针为正**（REP-103）。
注意下位机底层控制使用顺时针为正，仅在驱动层做了符号转换。

---

## 话题

| 话题 | 类型 | 发布者 |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | `lslidar_driver` |
| `/imu/data` | `sensor_msgs/Imu` | `uart_imu` |
| `/odom` | `nav_msgs/Odometry` | `uart_imu` |
| `/odometry/filtered` | `nav_msgs/Odometry` | `ekf_node` |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | EKF / 静态 TF / slam_toolbox |

---

## 相关仓库

| 仓库 | 内容 |
|---|---|
| [**six_robot**](https://github.com/luozhenyefly-wq/six_robot) | 六足机器人下位机固件、硬件设计、机械结构 |
| **six_robot_slam**（本仓库） | 上位机 ROS 2 SLAM 程序 |

---

## 许可

本仓库自有代码采用 **MIT License**，见 [LICENSE](LICENSE)。

`src/lslidar_driver/` 与 `src/lslidar_msgs/` 为镭神智能（Leishen Intelligent System）
提供的第三方 ROS 2 包，各自保留原有许可证，详见包内 `package.xml`。

---

## 开发说明

本项目在 RDK X5 上开发与验证，板端算力有限（3 GB 内存），部分配置针对该平台做了
取舍（如限制雷达量程、降低地图更新频率）。详细取舍与踩坑记录见
[docs/05_troubleshooting.md](docs/05_troubleshooting.md)。
