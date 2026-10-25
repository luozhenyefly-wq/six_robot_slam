# six_robot_slam

[English](README.md) | **中文**

> 六足机器人（six_robot）的 **Linux 上位机 ROS 2 程序** —— 激光雷达 + 串口 IMU/里程计的
> SLAM 建图与定位框架，运行于 D-Robotics RDK X5（aarch64 / Ubuntu 22.04 / ROS 2 Humble）。

---

## 与 six_robot 仓库的关系

本项目是六足机器人系统的**两个仓库之一**，与下位机仓库
[**luozhenyefly-wq/six_robot**](https://github.com/luozhenyefly-wq/six_robot) 配套使用：

```
                    ┌─────────────────────────────────────────┐
                    │   luozhenyefly-wq/six_robot             │
                    │   下位机仓库                             │
                    │   STM32 固件 / 硬件设计 / 机械结构       │
                    └───────────────────┬─────────────────────┘
                                        │ 串口 UART (115200 8N1)
                                        │ 0xAA | yaw | vyaw | odom | 0xBB
                                        ▼
                    ┌─────────────────────────────────────────┐
                    │   luozhenyefly-wq/six_robot_slam        │
                    │   本仓库 · 上位机                       │
                    │   ROS 2 SLAM / 定位 / 导航              │
                    └─────────────────────────────────────────┘
```

**两者通过一根串口线连接，由下位机单向上报传感器数据，本仓库只读不写。**

### 分工

| 维度 | `six_robot`（下位机） | `six_robot_slam`（本仓库 · 上位机） |
|---|---|---|
| **软件** | STM32 固件（C / HAL） | ROS 2 节点（C++ / Python） |
| **运行平台** | STM32 微控制器（裸机 / RTOS） | RDK X5，Ubuntu 22.04 |
| **核心职责** | 电机控制、步态解算、IMU 采集、里程累加 | 传感器接入、多源融合、SLAM、导航 |
| **硬件内容** | PCB、原理图、机械结构、装配说明 | — |
| **对外输出** | 串口数据帧（yaw / vyaw / 里程） | ROS 2 话题 / TF / 地图 |
| **实时性要求** | **硬实时**（步态周期不能抖） | 软实时（建图慢一点不影响行走） |
| **能否独立工作** | ✅ 可独立行走（不带建图也能遥控） | ❌ 依赖下位机提供 IMU 与里程 |

### 数据流向（单向）

```
下位机 STM32  ──串口帧──▶  uart_imu 节点  ──▶  /imu/data + /odom
                                                    │
                                  激光雷达 ──▶ /scan │
                                                    ▼
                                          EKF 融合 ──▶ SLAM 建图 ──▶ /map
```

**为什么不放在一个仓库里**

两边**工具链、编译方式、发布节奏完全不同**：STM32 用 Keil / CubeIDE 编译烧录，
本仓库用 `colcon build`；改步态参数不需要动上位机，改 SLAM 参数也不需要重烧固件。
分仓可以各自独立迭代，互不产生构建噪声。

**唯一的耦合点是串口协议**。协议变更时**两个仓库必须同步更新**，字段定义见下文
[硬件依赖](#硬件依赖) 与 [docs/01_overview.md](docs/01_overview.md)。

---

## 这是什么

本仓库是六足机器人系统的**上位机部分**，负责：

- 读取 **镭神 N10-P 激光雷达** → 发布 `/scan`
- 读取 **STM32 下位机串口帧**（yaw / 偏航角速度 / 里程增量）→ 发布 `/imu/data` 与 `/odom`
- 用 **robot_localization (EKF)** 融合 IMU 与足式里程计 → 发布 `odom → base_footprint` TF
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
│   │       ├── slam_toolbox.yaml        建图参数
│   │       └── slam.rviz                rviz 预设（QoS 已配好）
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
├── README.md                 English
└── README.zh-CN.md           中文（本文件）
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

| 仓库 | 角色 | 内容 |
|---|---|---|
| [**six_robot**](https://github.com/luozhenyefly-wq/six_robot) | 下位机 | STM32 固件、PCB 硬件设计、机械结构 |
| **six_robot_slam**（本仓库） | 上位机 | ROS 2 传感器接入、EKF 融合、SLAM、导航 |

两个仓库合起来才是完整的六足机器人系统。关系与分工详见上文
[与 six_robot 仓库的关系](#与-six_robot-仓库的关系)。

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
