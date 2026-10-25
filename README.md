# six_robot_slam

**English** | [中文](README.zh-CN.md)

> **Linux onboard (host-side) ROS 2 program** for the six-legged robot *six_robot* — a 2D
> SLAM / localization framework built on a LiDAR plus a serial IMU/odometry feed, running on
> D-Robotics RDK X5 (aarch64 / Ubuntu 22.04 / ROS 2 Humble).

---

## Relationship to the six_robot repository

This project is **one of two repositories** that together form the six-legged robot system.
It works together with the firmware/hardware repository
[**luozhenyefly-wq/six_robot**](https://github.com/luozhenyefly-wq/six_robot):

```
                    ┌─────────────────────────────────────────┐
                    │   luozhenyefly-wq/six_robot             │
                    │   FIRMWARE REPOSITORY                   │
                    │   STM32 firmware / hardware / mechanics │
                    └───────────────────┬─────────────────────┘
                                        │ UART serial (115200 8N1)
                                        │ 0xAA | yaw | vyaw | odom | 0xBB
                                        ▼
                    ┌─────────────────────────────────────────┐
                    │   luozhenyefly-wq/six_robot_slam        │
                    │   THIS REPOSITORY · host-side           │
                    │   ROS 2 SLAM / localization / nav       │
                    └─────────────────────────────────────────┘
```

**The two halves are joined by a single serial cable.** The lower-level controller only
reports data upstream; this repository only reads it and never writes back.

### Division of responsibilities

| Aspect | `six_robot` (lower level) | `six_robot_slam` (this repo · host) |
|---|---|---|
| **Software** | STM32 firmware (C / HAL) | ROS 2 nodes (C++ / Python) |
| **Platform** | STM32 MCU (bare metal / RTOS) | RDK X5, Ubuntu 22.04 |
| **Core duty** | Motor control, gait solving, IMU sampling, odometry accumulation | Sensor ingest, multi-source fusion, SLAM, navigation |
| **Hardware content** | PCB, schematics, mechanics, assembly docs | — |
| **Output** | Serial data frames (yaw / vyaw / odometry) | ROS 2 topics / TF / maps |
| **Timing** | **Hard real-time** (gait cycle must not jitter) | Soft real-time (slow mapping does not affect walking) |
| **Runs standalone** | ✅ Can walk and be driven without mapping | ❌ Needs IMU + odometry from the lower level |

### Data flow (one-way)

```
Lower level STM32 ──serial frames──▶ uart_imu node ──▶ /imu/data + /odom
                                                          │
                              LiDAR ──▶ /scan             │
                                                          ▼
                                    EKF fusion ──▶ SLAM ──▶ /map
```

**Why keep two repositories**

The two sides have **completely different toolchains, build systems and release cadence**:
the STM32 is built and flashed with Keil / CubeIDE, while this repository is built with
`colcon build`. Changing gait parameters does not require touching the host side, and
changing SLAM parameters does not require re-flashing firmware. Keeping them separate lets
each iterate independently with no build noise.

**The only coupling point is the serial protocol.** When the protocol changes, **both
repositories must be updated together**. Field definitions are in
[Hardware](#hardware) below and in [docs/01_overview.md](docs/01_overview.md) *(Chinese)*.

---

## What this repository does

This is the **host-side half** of the six-legged robot system. It:

- Reads the **Leishen N10-P LiDAR** → publishes `/scan`
- Reads **STM32 serial frames** (yaw / yaw rate / odometry increments) → publishes `/imu/data` and `/odom`
- Fuses IMU with legged odometry using **robot_localization (EKF)** → publishes the `odom → base_footprint` TF
- Runs 2D LiDAR SLAM with **slam_toolbox** → publishes the `map → odom` TF and the `/map` occupancy grid
- Provides a **Nav2** entry point (launch file present, parameters still to be written)

Full TF tree:

```
map ──(slam_toolbox)──> odom ──(EKF)──> base_footprint ──(static)──> base_link ──(static)──> laser_link
                                                                  └─(static)──> imu_link
```

Every TF segment has exactly one publisher, following REP-105.

---

## Repository layout

```
robot_ws/
├── src/
│   ├── robot_bringup/        Launch orchestration: sensors / localization / slam / nav2
│   │   ├── launch/
│   │   │   ├── robot.launch.py          Main entry point
│   │   │   ├── sensors.launch.py        uart_imu + LiDAR
│   │   │   ├── localization.launch.py   EKF + static TF
│   │   │   ├── slam.launch.py           slam_toolbox
│   │   │   └── nav2.launch.py           Nav2 (reserved)
│   │   └── config/
│   │       ├── ekf.yaml                 robot_localization parameters
│   │       ├── slam_toolbox.yaml        SLAM parameters
│   │       └── slam.rviz                Pre-configured rviz layout (QoS already matched)
│   │
│   ├── uart_imu/             C++ serial node: frame parsing → Imu + Odometry
│   ├── demo_cpp_tf/          Static / dynamic TF broadcast and listen examples
│   ├── lslidar_driver/       Leishen LiDAR driver (third-party)
│   ├── lslidar_msgs/         Leishen LiDAR message definitions (third-party)
│   └── robot_navigation2/    Map and navigation resources (nav not enabled yet)
│
├── docs/                     (Chinese)
│   ├── 01_overview.md        Project overview: architecture / TF / topics / hardware / protocol
│   ├── 02_environment.md     Environment: ROS 2 Humble + slam_toolbox + rviz
│   ├── 03_build.md           Build instructions
│   ├── 04_run_slam.md        Mapping workflow
│   └── 05_troubleshooting.md Troubleshooting
│
├── LICENSE                   MIT (third-party packages excluded)
├── README.md                 English (this file)
└── README.zh-CN.md           Chinese
```

---

## Quick start

```bash
# 1. Set up the environment (see docs/02_environment.md)
sudo apt install ros-humble-desktop ros-humble-robot-localization \
                 ros-humble-slam-toolbox ros-humble-nav2-bringup

# 2. Build
cd ~/robot_ws
colcon build --symlink-install

# 3. Source
source /opt/ros/humble/setup.bash
source install/setup.bash

# 4. Start mapping
ros2 launch robot_bringup robot.launch.py slam:=true

# 5. Visualize in another terminal
rviz2 -d src/robot_bringup/config/slam.rviz
```

Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

---

## Hardware

| Part | Model | Interface | Notes |
|---|---|---|---|
| Host computer | D-Robotics RDK X5 | — | aarch64, Ubuntu 22.04, ROS 2 Humble |
| LiDAR | Leishen N10-P | USB (`/dev/ttyACM0`) | 2D single-line, ~10 m range, 400000 baud |
| Lower controller | STM32 | UART (`/dev/ttyS1`) | 115200 8N1, custom frame protocol |
| IMU | Integrated in lower controller | same UART | 6-axis relative angle, zeroed at power-on |

Serial frame protocol (lower level → host):

```
0xAA | yaw (int16, big-endian) | vyaw (int16, big-endian) | ... | 0xBB
```

- `yaw`: unit **0.01°**, relative angle, reset to zero on power-up
- `vyaw`: unit **0.01°/s**
- Odometry: unit **cm**

Coordinate convention: **x forward, y left, z up**; ROS angles are **counter-clockwise
positive** (REP-103). Note the lower-level controller uses clockwise-positive internally;
the sign conversion happens only in the driver layer.

---

## Topics

| Topic | Type | Publisher |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | `lslidar_driver` |
| `/imu/data` | `sensor_msgs/Imu` | `uart_imu` |
| `/odom` | `nav_msgs/Odometry` | `uart_imu` |
| `/odometry/filtered` | `nav_msgs/Odometry` | `ekf_node` |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | EKF / static TF / slam_toolbox |

---

## Related repositories

| Repository | Role | Contents |
|---|---|---|
| [**six_robot**](https://github.com/luozhenyefly-wq/six_robot) | Lower level | STM32 firmware, PCB hardware design, mechanical structure |
| **six_robot_slam** (this repo) | Host | ROS 2 sensor ingest, EKF fusion, SLAM, navigation |

Both repositories together form the complete six-legged robot system. See
[Relationship to the six_robot repository](#relationship-to-the-six_robot-repository) above.

---

## License

Original code in this repository is released under the **MIT License**, see [LICENSE](LICENSE).

`src/lslidar_driver/` and `src/lslidar_msgs/` are third-party ROS 2 packages provided by
Leishen Intelligent System and retain their own licenses; see each package's `package.xml`.

---

## Development notes

This project was developed and validated on the RDK X5. The board has limited resources
(3 GB usable RAM), so some configuration choices are tailored to that platform (e.g. a
capped LiDAR range and a reduced map update interval). See
[docs/05_troubleshooting.md](docs/05_troubleshooting.md) *(Chinese)* for the trade-offs and
the issues encountered.
