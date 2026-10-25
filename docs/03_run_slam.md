# 建图运行流程

## 一、硬件连接

```
┌──────────┐   USB (ACM0)   ┌──────────┐
│  N10-P   ├───────────────>│          │
│  激光雷达 │  400000 baud   │  RDK X5  │
└──────────┘                │          │
                            │          │
┌──────────┐   UART1 (S1)   │          │
│  STM32   ├───────────────>│          │
│  下位机   │  115200 8N1    └──────────┘
└──────────┘
```

确认设备:

```bash
ls -l /dev/ttyACM0     # 雷达
ls -l /dev/ttyS1       # 下位机串口
```

---

## 二、一键启动

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash

ros2 launch robot_bringup robot.launch.py slam:=true
```

这一条命令会依次拉起:

```
uart_imu            → /imu/data, /odom
lslidar_driver      → /scan
ekf_node            → odom → base_footprint (TF), /odometry/filtered
static_tf           → base_footprint → base_link → laser_link / imu_link
slam_toolbox        → map → odom (TF), /map
```

### launch 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `slam` | `false` | `true` 启用 slam_toolbox 建图 |
| `nav` | `false` | `true` 启用 Nav2(需提供地图) |
| `map` | `""` | Nav2 使用的地图 yaml 路径 |
| `use_sim_time` | `false` | 真机必须 false |
| `base_link_z` | `0.06` | `base_footprint` → `base_link` 高度(米) |
| `imu_port` | `/dev/ttyS1` | 下位机串口 |
| `imu_baudrate` | `115200` | 下位机波特率 |

---

## 三、看地图

另开终端:

```bash
source ~/robot_ws/install/setup.bash
rviz2 -d ~/robot_ws/src/robot_bringup/config/slam.rviz
```

**rviz 配置要点**(详见 01_environment.md §5):

- LaserScan `/scan` → Reliability **Best Effort**
- Map `/map` → Durability **Transient Local**
- Fixed Frame 先设 `map`

---

## 四、检查数据是否正常

**开跑后的前 30 秒,建议逐项确认:**

```bash
# 1) 话题都在
ros2 topic list | grep -E "scan|imu|odom|map"

# 2) 雷达有数据(需 best_effort)
ros2 topic hz /scan --qos-reliability best_effort
# 期望 ~10 Hz

# 3) IMU / 里程计
ros2 topic hz /imu/data
ros2 topic hz /odom
# 期望 ~20-50 Hz(取决于下位机发送频率)

# 4) EKF 输出
ros2 topic hz /odometry/filtered
# 期望 ~20-30 Hz

# 5) TF 链完整
ros2 run tf2_ros tf2_echo map base_footprint
```

### 判断标准

| 现象 | 含义 |
|---|---|
| `/scan` 有但 rviz 空 | QoS 不匹配 → 改 Best Effort |
| `/map` 有但 rviz 空 | 缺 Transient Local |
| TF 报 `Frame [xxx] does not exist` | 上游节点没起来 |
| 静止时 yaw 一直漂 | 陀螺零偏,需重标定 |

---

## 五、推着机器人走

建图过程中:

- **速度慢一点**(0.2~0.3 m/s),转圈慢
- **走闭环**:绕房间一圈回到起点,让 slam 有机会做回环检测
- **避免长时间原地转**(2D 激光 SLAM 对纯旋转不友好)
- 每 **2~3 分钟**看一次 rviz,确认没有明显错位

如果地图开始"重影"或墙体变厚:

1. 先停下,不要继续走
2. 看 rviz 里 `map → odom` TF 是否在跳
3. 检查 `/odom` 是否漂(见 04_troubleshooting.md)

---

## 六、保存地图

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/room
```

会生成两个文件:

```
~/maps/room.pgm     # 栅格图
~/maps/room.yaml    # 元数据(分辨率、原点、阈值)
```

把这两个文件放到 `src/robot_navigation2/src/map/` 下,连同 yaml 一起提交。

---

## 七、定位模式(已有地图)

等地图和 Nav2 参数就绪后:

```bash
ros2 launch robot_bringup robot.launch.py nav:=true map:=~/maps/room.yaml
```

或者在 slam_toolbox 里切换为定位模式(修改 `slam_toolbox.yaml` 的 `mode: localization`)。
这部分目前**尚未完成**,需要补 `nav2_params.yaml`。

---

## 八、关闭

```bash
# 在 launch 终端按 Ctrl+C
# 若节点残留(串口被占),按 PID 精确 kill:
pgrep -af "uart_imu_node|lslidar_driver_node|ekf_node|slam_toolbox"
kill -9 <pid>
```

> **不要用 `pkill -f xxx`** —— 它可能匹配到执行命令的那个 shell,把自己杀掉。
