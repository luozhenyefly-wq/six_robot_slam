# 环境配置:ROS 2 Humble + slam_toolbox + rviz

本文记录本项目的完整运行环境搭建过程。已在 **RDK X5(aarch64 / Ubuntu 22.04)** 上验证。

> 在 x86 Ubuntu 22.04 上同样适用,仅需把 arm64 相关说明忽略。

---

## 1. 系统要求

| 项 | 要求 |
|---|---|
| OS | Ubuntu 22.04 (jammy) |
| ROS 2 | Humble Hawksbill |
| 架构 | aarch64(本机)或 x86_64 |
| 内存 | ≥ 2 GB(建议 4 GB;i2c 视频/BPU 会占用部分内存) |
| 磁盘 | ≥ 20 GB |

---

## 2. 安装 ROS 2 Humble

### 2.1 配置 apt 源(国内推荐清华镜像)

```bash
sudo apt install -y software-properties-common curl gnupg lsb-release

# GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# 添加地址源(清华镜像,国内速度快)
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

### 2.2 安装桌面版(含 rviz2、rqt)

```bash
sudo apt install -y ros-humble-desktop
```

`ros-humble-desktop` 包含:
- `rviz2` — 3D 可视化
- `rqt` 系列工具
- 常用的 `demo_nodes`、`turtlesim` 等

> 若空间紧张,可只装 `ros-humble-ros-base` + `ros-humble-rviz2`,但本项目推荐 desktop。

### 2.3 加载环境

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

验证:

```bash
ros2 --version
# 或
ros2 run demo_nodes_cpp talker    # 另开终端 ros2 run demo_nodes_cpp listener
```

---

## 3. 安装本项目依赖

```bash
sudo apt install -y \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  ros-humble-nav2-bringup \
  ros-humble-nav2-map-server
```

| 包 | 用途 |
|---|---|
| `robot-localization` | EKF/UKF 多传感器融合,发布 `odom → base_footprint` |
| `slam-toolbox` | 2D 激光 SLAM,发布 `map → odom` 与 `/map` |
| `nav2-bringup` | 导航栈(本项目预留,参数待补) |
| `nav2-map-server` | 提供 `map_saver_cli` 保存地图 |

编译工具:

```bash
sudo apt install -y python3-colcon-common-extensions
```

---

## 4. slam_toolbox 配置说明

本项目使用 **异步模式**(`async_slam_toolbox_node`),配置文件:

```
src/robot_bringup/config/slam_toolbox.yaml
```

关键参数与取舍:

```yaml
slam_toolbox:
  ros__parameters:
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint      # ★ 与 EKF 输出一致
    scan_topic: /scan

    mode: mapping                   # mapping | localization

    # —— 板端算力有限,以下做了放宽 ——
    transform_timeout: 1.0          # ★ 默认 0.2;TF 抖动时会大量丢帧
    minimum_time_interval: 0.1      # 处理间隔(s)
    map_update_interval: 5.0        # 地图刷新间隔(s),降低 CPU 占用
    throttle_scans: 1               # 每帧都处理;CPU 紧张可调成 2

    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000     # 40 MB,避免 ceres 小栈溢出
    solver_plugin: solver_plugins::CeresSolver
```

**为什么 `transform_timeout` 要调到 1.0**

真机上 EKF 输出 `odom → base_footprint` 的频率(约 20 Hz)低于雷达扫描频率(10 Hz)。
默认 0.2 s 的超时在板端 CPU 抖动时很容易踩空,导致 slam_toolbox 报:

```
Message Filter dropping message: frame 'laser_link' at time ... for reason 'the timestamp on the message is earlier than all the data in the transform cache'
```

把它放宽到 1.0 s 后大幅减少。注意这**只影响"等 TF 的耐心"**,不影响地图精度。

**关于 `Message Filter dropping ... queue is full`**

在低算力设备上,如果 slam 处理速度跟不上扫描速度,会丢旧帧。**这不影响地图生成质量**,
因为每帧激光之间只有几度的角度差。若确实需要减少,可以:

- 提高 `minimum_time_interval`(少处理几帧)
- 降低雷达转速/扫描频率
- 关闭 `throttle_scans`

---

## 5. rviz2 配置

### 5.1 启动

```bash
rviz2 -d src/robot_bringup/config/slam.rviz
```

### 5.2 必需的显示项与 QoS

| Display | Topic | 注意 |
|---|---|---|
| **LaserScan** | `/scan` | Reliability 设为 **Best Effort** |
| **Map** | `/map` | Durability 设为 **Transient Local**,Reliability **Reliable** |
| **TF** | — | 用于看坐标系是否正确 |
| **RobotModel** | — | 可选,需先有 URDF |

**QoS 不对会表现为"话题有数据但 rviz 里空"** —— 这是最常见的新手坑。

### 5.3 固定坐标系

`Fixed Frame` 建图时设为 **`map`**;**建图尚未启动时**设为 `odom` 或 `base_footprint`,
否则 rviz 会报 `Fixed Frame [map] does not exist`。

---

## 6. 串口权限

```bash
# 把自己加入 dialout 组,免 sudo 访问串口
sudo usermod -aG dialout $USER
# 重新登录生效
```

验证:

```bash
ls -l /dev/ttyS1 /dev/ttyACM0
# crw-rw---- 1 root dialout ...   ← 组是 dialout 即可
```

---

## 7. 交叉检查清单

装完后应当满足:

```bash
# ROS 2 可用
ros2 pkg list | grep -E "slam_toolbox|robot_localization|nav2_map_server"

# 雷达设备
ls -l /dev/ttyACM0

# 下位机串口
ls -l /dev/ttyS1

# 波特率(雷达为 400000)
stty -F /dev/ttyS1 115200 raw -echo && echo "UART OK"
```

---

## 8. 免代理安装说明

本项目所有依赖均可通过**国内镜像**安装,无需代理:

- ROS 2 包 → 清华镜像 `https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu`
- 系统包 → `http://ports.ubuntu.com`(aarch64)或 `http://archive.ubuntu.com`

若确实需要代理,只有访问 GitHub(如源码编译)时才需要,且注意 **apt 的代理配置是独立的**:

```bash
# /etc/apt/apt.conf.d/99proxy
Acquire::http::Proxy "http://<proxy-ip>:<port>";
Acquire::https::Proxy "http://<proxy-ip>:<port>";
```

`git` 则用:

```bash
git config --global http.proxy http://<proxy-ip>:<port>
git config --global https.proxy http://<proxy-ip>:<port>
```

---

## 9. 参考

- [ROS 2 Humble 官方安装文档](https://docs.ros.org/en/humble/Installation.html)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [robot_localization](http://docs.ros.org/en/humble/p/robot_localization/)
- [REP-105 Coordinate Frames](https://www.ros.org/reps/rep-0105.html)
