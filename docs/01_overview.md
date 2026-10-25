# 项目说明

> 六足机器人（six_robot）**Linux 上位机**的 ROS 2 SLAM 程序。
> 本文说明这个项目**是什么、怎么组织、各模块做什么**。
>
> 想跑起来请看 [02_environment.md](02_environment.md)（环境配置）→ [03_build.md](03_build.md)（编译）→ [04_run_slam.md](04_run_slam.md)（运行）。

---

## 1. 项目定位

本仓库是六足机器人系统的**上位机部分**，运行在机器人板载 Linux 主机上
（本项目在 **D-Robotics RDK X5 / aarch64 / Ubuntu 22.04 / ROS 2 Humble** 上开发验证）。

**下位机（STM32）固件与硬件工程在另一个仓库：**
[luozhenyefly-wq/six_robot](https://github.com/luozhenyefly-wq/six_robot)

职责划分：

| 层 | 负责 |
|---|---|
| **下位机（STM32）** | 电机控制、步态解算、IMU 采集、里程累加 → 通过串口上报 |
| **上位机（本仓库）** | 读串口与雷达、传感器融合、SLAM 建图、导航决策 |

---

## 2. 功能概览

上位机做四件事：

1. **采集** — 读镭神 N10-P 激光雷达 → `/scan`；读 STM32 串口帧 → `/imu/data`、`/odom`
2. **融合** — robot_localization (EKF) 融合 IMU 与里程计 → `odom → base_footprint` TF
3. **建图** — slam_toolbox 做 2D 激光 SLAM → `map → odom` TF、`/map` 栅格地图
4. **导航**（预留）— Nav2 接口已就位，参数待补

---

## 3. 系统架构

```
┌──────────────────────────────────────────────────────────────┐
│                    RDK X5  (Ubuntu 22.04 / ROS 2 Humble)     │
│                                                              │
│  /dev/ttyACM0                        /dev/ttyS1              │
│      │                                   │                   │
│  ┌───▼──────────────┐            ┌───────▼──────────────┐    │
│  │ lslidar_driver   │            │ uart_imu  (C++)      │    │
│  │ 镭神 N10-P 驱动   │            │ 串口帧解析            │    │
│  └───┬──────────────┘            └───┬──────────┬───────┘    │
│      │ /scan                         │/imu/data │/odom       │
│      │                               │          │            │
│      │                          ┌────▼──────────▼────┐       │
│      │                          │  ekf_node (EKF)    │       │
│      │                          │  robot_localization│       │
│      │                          └────────┬───────────┘       │
│      │                                   │ odom→base_footprint│
│  ┌───▼───────────────────────────────────▼───────────┐       │
│  │              slam_toolbox (async)                 │       │
│  │  2D 激光 SLAM  →  map→odom TF + /map              │       │
│  └───────────────────────┬───────────────────────────┘       │
│                          │                                   │
│                    ┌─────▼─────┐                             │
│                    │   rviz2   │  可视化                     │
│                    └───────────┘                             │
└──────────────────────────────────────────────────────────────┘
```

### 3.1 TF 树

```
map ──(slam_toolbox)──> odom ──(ekf_node)──> base_footprint
                                                  │
                                          (static_tf_broadcaster)
                                                  │
                                    ┌─────────────┴─────────────┐
                                    ▼                           ▼
                                base_link                    imu_link
                                    │
                              (static TF)
                                    ▼
                                laser_link
```

**每段 TF 只有一个发布者**，符合 REP-105：

| 段 | 发布者 | 说明 |
|---|---|---|
| `map → odom` | `slam_toolbox` | 建图/定位纠偏 |
| `odom → base_footprint` | `ekf_node` | 融合后的连续位姿 |
| `base_footprint → base_link` | `static_tf_broadcaster` | 机身高度 `z = base_link_z`(默认 0.06 m) |
| `base_link → laser_link` | `static_tf_broadcaster` | 雷达安装位置 + 朝向 |
| `base_link → imu_link` | `static_tf_broadcaster` | IMU 安装位置 |

**为什么用 `base_footprint` 而不是直接到 `base_link`**

足式机器人行走时机身有起伏和俯仰晃动。把 EKF 输出接到 `base_footprint`（地面投影点），
机身高度和姿态变化由静态 TF 吸收，2D SLAM 能得到干净的地面平面假设。

### 3.2 坐标系约定

- 机体：**x 前、y 左、z 上**（ROS 右手系）
- 角度：**逆时针为正**（REP-103）
- 下位机底层控制使用**顺时针为正**，仅在 `uart_imu` 驱动层做符号转换

---

## 4. 包结构

```
robot_ws/
└── src/
    ├── robot_bringup/             启动编排 + 配置（无源码）
    │   ├── launch/
    │   │   ├── robot.launch.py          总入口
    │   │   ├── sensors.launch.py        uart_imu + 雷达
    │   │   ├── localization.launch.py   EKF + 静态 TF
    │   │   ├── slam.launch.py           slam_toolbox
    │   │   └── nav2.launch.py           Nav2（预留）
    │   └── config/
    │       ├── ekf.yaml                 robot_localization 参数
    │       ├── slam_toolbox.yaml        建图参数
    │       └── slam.rviz                rviz 预设（QoS 已配好）
    │
    ├── uart_imu/                   C++ 串口节点
    │   └── src/uart_imu_node.cpp        帧解析 → Imu + Odometry
    │
    ├── demo_cpp_tf/                TF 示例
    │   ├── static_tf_broadcaster.cpp    静态 TF（本项目实际使用）
    │   ├── dynamic_tf_broadcaster.cpp   动态 TF 示例
    │   └── tf_listener.cpp              TF 查询示例
    │
    ├── lslidar_driver/             镭神雷达驱动（第三方）
    ├── lslidar_msgs/               雷达消息定义（第三方）
    └── robot_navigation2/          地图资源
```

### 4.1 各包职责

| 包 | 类型 | 说明 |
|---|---|---|
| `robot_bringup` | `ament_cmake` | 纯 launch + yaml，不编译源码。总入口是 `robot.launch.py` |
| `uart_imu` | `ament_cmake` | termios 原生串口（无第三方依赖），一帧解析出 yaw/vyaw/里程 |
| `demo_cpp_tf` | `ament_cmake` | 静态 TF 广播是**生产中实际使用**的；动态广播与监听是学习示例 |
| `lslidar_driver` | `ament_cmake` | 镭神官方驱动，本项目仅改 `params/lsx10.yaml` |
| `lslidar_msgs` | `ament_cmake` | 雷达自定义消息 |
| `robot_navigation2` | `ament_cmake` | 存地图（当前有 `map1.pgm/yaml`），导航启动待补 |

---

## 5. 话题

| 话题 | 类型 | 发布者 | 频率 |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | `lslidar_driver` | ~10 Hz |
| `/imu/data` | `sensor_msgs/Imu` | `uart_imu` | 取决于下位机 |
| `/odom` | `nav_msgs/Odometry` | `uart_imu` | 取决于下位机 |
| `/odometry/filtered` | `nav_msgs/Odometry` | `ekf_node` | 30 Hz |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` | 5 s 一次 |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | 见 TF 树 | — |

**QoS 注意（消费端）**

| 话题 | 需要的 QoS |
|---|---|
| `/scan` | Reliability = **Best Effort** |
| `/imu/data`、`/odom` | Reliability = **Best Effort** |
| `/map` | Durability = **Transient Local**, Reliability = Reliable |
| `/tf_static` | Durability = **Transient Local** |

订阅端 QoS 不匹配时表现为“话题有数据但收不到”。

---

## 6. 硬件

| 部件 | 型号 | 接口 | 参数 |
|---|---|---|---|
| 主控 | D-Robotics RDK X5 | — | aarch64, Ubuntu 22.04, 3 GB 可用内存 |
| 激光雷达 | 镭神 **N10-P** | `/dev/ttyACM0` | 2D 单线，**400000** 波特率，10 m 级 |
| 下位机 | STM32 | `/dev/ttyS1` (UART1) | **115200 8N1** |
| IMU | 集成于下位机 | 同上 | 六轴相对角度，**上电即零点** |

> **IMU 是无绝对参考相对角度**：零点取决于上电时的朝向，没有磁力计。
> 这意味着每次上电后的 yaw 起点不同，靠 SLAM 的 `map → odom` 来纠正绝对朝向。

### 6.1 雷达安装参数

雷达相对机体的安装位姿配置在 `demo_cpp_tf/launch/static_tf.launch.py`：

```
base_link → laser_link :  x=0.030, y=0.0, z=0.096, rpy=(0, 0, 1.828 rad)
```

`yaw = 1.828 rad (≈104.7°)` 是因为雷达的 +x 轴与机体 +y 方向成 14.7° 夹角，
需要旋转到机体坐标系。**换车或改安装后必须重新核对这个值**，否则点云方向会整体偏转。

### 6.2 串口帧协议（下位机 → 上位机）

```
┌──────┬────────────┬─────────────┬──────────┬──────┐
│ 0xAA │ yaw (i16)  │ vyaw (i16)  │ odom...  │ 0xBB │
└──────┴────────────┴─────────────┴──────────┴──────┘
        大端序        大端序
```

| 字段 | 类型 | 单位 | 说明 |
|---|---|---|---|
| 帧头 | `uint8` | — | 固定 `0xAA` |
| `yaw` | `int16` 大端 | **0.01°** | 相对角度，上电清零 |
| `vyaw` | `int16` 大端 | **0.01°/s** | 偏航角速度 |
| 里程 | — | **cm** | 位置增量，需 × 0.01 转米 |
| 帧尾 | `uint8` | — | 固定 `0xBB` |

解析策略：找 `0xAA` 帧头 → 校验帧尾 `0xBB` → 不匹配则丢字节重同步。

**转换关系（在 `uart_imu_node.cpp` 内完成）**

```
yaw   : int16 → × 0.01 → 度 → × π/180 → 弧度 → 四元数（绕 Z）
vyaw  : int16 → × 0.01 → 度/秒 → × π/180 → 弧度/秒
odom  : cm → × 0.01 → 米
```

---

## 7. 关键参数与取舍

### 7.1 EKF（`config/ekf.yaml`）

```yaml
frequency: 30.0                 # 输出频率
odom_frame: odom                # 世界系
base_link_frame: base_footprint # ★ 直接输出到 base_footprint
world_frame: odom               # = odom_frame ⇒ 发 odom → base_footprint
sensor_timeout: 0.3             # 传感器无数据多久后停止融合
```

**为什么把 `base_link_frame` 设为 `base_footprint`**

EKF 默认发 `world_frame → base_link_frame`。设成 `base_footprint` 后，
`base_footprint → base_link` 交给静态 TF，避免与 SLAM 的 2D 平面假设冲突。

### 7.2 slam_toolbox（`config/slam_toolbox.yaml`）

```yaml
base_frame: base_footprint      # ★ 必须与 EKF 输出一致
mode: mapping                   # mapping | localization
transform_timeout: 1.0          # ★ 默认 0.2，板端需放宽
map_update_interval: 5.0        # 降低 CPU 占用
minimum_time_interval: 0.1
throttle_scans: 1
tf_buffer_duration: 30.0
stack_size_to_use: 40000000     # 40 MB
```

| 参数 | 为什么这么调 |
|---|---|
| `transform_timeout: 1.0` | 默认 0.2 s 在低算力板子上很容易超时，导致大量 `Message Filter dropping` |
| `map_update_interval: 5.0` | 地图刷新太频繁会把 CPU 吃满，5 秒足够 |
| `stack_size_to_use: 40000000` | ceres 求解器需要较大栈，用小栈会段错误 |

> **`transform_timeout` 只影响“等 TF 的耐心”，不影响地图精度。**

### 7.3 雷达量程（`lslidar_driver/params/lsx10.yaml`）

```yaml
lidar_name: N10_P
max_range: 11.0     # N10-P 是 10 m 级，留 1 m 余量
```

**限制量程的作用**：滤掉超量程的反射假点和多径干扰，减少无效点云对 slam 匹配的干扰。

---

## 8. 启动方式

总入口是 `robot_bringup/launch/robot.launch.py`，参数组合控制启用哪些功能：

```bash
# 只起传感器 + EKF
ros2 launch robot_bringup robot.launch.py

# 加建图
ros2 launch robot_bringup robot.launch.py slam:=true

# 加导航（需地图，参数待补）
ros2 launch robot_bringup robot.launch.py nav:=true map:=/path/to/map.yaml
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `slam` | `false` | 启用 slam_toolbox |
| `nav` | `false` | 启用 Nav2 |
| `map` | `""` | Nav2 地图路径 |
| `use_sim_time` | `false` | 真机必须 false |
| `base_link_z` | `0.06` | `base_footprint → base_link` 高度（m） |
| `imu_port` | `/dev/ttyS1` | 下位机串口 |
| `imu_baudrate` | `115200` | 下位机波特率 |

详细运行流程见 [04_run_slam.md](04_run_slam.md)。

---

## 9. 可视化

`src/robot_bringup/config/slam.rviz` 是预配置好的 rviz 布局，**QoS 已经配对**：

```bash
rviz2 -d ~/robot_ws/src/robot_bringup/config/slam.rviz
```

包含图层：

| 图层 | 话题 | QoS |
|---|---|---|
| Grid | — | — |
| **Map** | `/map` | Transient Local + Reliable |
| **LaserScan** | `/scan` | Best Effort |
| **TF** | — | — |
| **Odometry** | `/odometry/filtered` | Best Effort |

Fixed Frame = `map`（建图未启动时可临时改为 `odom`）。

---

## 10. 当前进度与待办

**已完成**

- [x] 雷达驱动接入（N10-P，10 Hz）
- [x] C++ 串口节点（IMU + 里程计）
- [x] EKF 融合，TF 树完整
- [x] slam_toolbox 建图跑通
- [x] rviz 预设

**待办**

- [ ] `nav2_params.yaml` 编写
- [ ] `robot_navigation2` 补 launch
- [ ] 定位模式（`mode: localization`）验证
- [ ] 里程计漂移标定（陀螺零偏、尺度、角度三项）
- [ ] 地图长期稳定性优化

---

## 11. 许可

本项目自有代码采用 **MIT License**，见 [LICENSE](../LICENSE)。

`src/lslidar_driver/` 与 `src/lslidar_msgs/` 为镭神智能提供的第三方 ROS 2 包，
各自保留原有许可证，详见包内 `package.xml`。

---

## 12. 相关仓库

| 仓库 | 内容 |
|---|---|
| [**six_robot**](https://github.com/luozhenyefly-wq/six_robot) | 六足机器人下位机固件、硬件设计、机械结构 |
| **six_robot_slam**（本仓库） | 上位机 ROS 2 SLAM 程序 |
