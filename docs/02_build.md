# 编译说明

## 前置条件

已完成 [01_environment.md](01_environment.md) 中的环境配置,并且:

```bash
source /opt/ros/humble/setup.bash
```

## 编译

```bash
cd ~/robot_ws
colcon build --symlink-install
```

`--symlink-install` 的好处:修改 Python launch / yaml 配置后**无需重新编译**。

### 只编译某个包

```bash
colcon build --packages-select robot_bringup
colcon build --packages-select uart_imu
```

### 编译后加载

**每次新开终端都要 source:**

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
```

建议写进 `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash 2>/dev/null
```

---

## 各包说明

| 包 | 类型 | 说明 |
|---|---|---|
| `robot_bringup` | `ament_cmake` | 纯 launch + 配置,无源码 |
| `uart_imu` | `ament_cmake` | C++ 串口节点 |
| `demo_cpp_tf` | `ament_cmake` | TF 广播/监听示例 |
| `lslidar_driver` | `ament_cmake` | 第三方雷达驱动 |
| `lslidar_msgs` | `ament_cmake` | 第三方消息定义 |
| `robot_navigation2` | `ament_cmake` | 地图资源 |

---

## ⚠️ 本机(RDK X5)编译注意事项

### 1. 内存紧张,串行编译更稳

板子只有 3 GB 内存(约 1 GB 被 CMA/BPU/GPU 保留),**并行编译容易 OOM**:

```bash
colcon build --symlink-install --executor sequential
```

或者限制并行度:

```bash
MAKEFLAGS="-j2" colcon build
```

### 2. 后台编译(避免 SSH 断开会终止)

```bash
setsid bash -c 'cd ~/robot_ws && colcon build --symlink-install > /tmp/build.log 2>&1' &
```

> **不要用 `nohup`** —— 在某些会话管理下会被回收,`setsid` 才能真脱离。

### 3. 编译很慢是正常的

`lslidar_driver` 在 X5 上首次编译约 **3 分钟**,全量编译 5~10 分钟。

### 4. 内存不够时可以加 swap

```bash
sudo fallocate -l 2G /swapfile && sudo chmod 600 /swapfile
sudo mkswap /swapfile && sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

---

## 编译成功判定

```
Summary: 4 packages finished [3min 3s]
  1 package had stderr output: lslidar_driver
```

- `lslidar_driver` 的 stderr 通常是 pcap 头文件的无害警告,**可以忽略**
- 只要有 `finished` 且没有 `Failed`,就说明成功了

失败时的排查:

```bash
# 看某个包的详细日志
cat log/latest_build/<包名>/stderr.log
```

---

## 常见编译错误

| 错误 | 原因 | 解决 |
|---|---|---|
| `Could not find a package configuration file provided by "rclcpp"` | 没 source ROS | `source /opt/ros/humble/setup.bash` |
| `undefined reference to Xxx::Xxx()` | `.cpp` 没加进 `CMakeLists.txt` | 在 `add_executable()` 里补上 |
| `fatal error: some_header.h: No such file` | 缺依赖包 | 检查 `package.xml` 有对应 `<depend>` |
| `error: ‘xxx’ was not declared` | 漏 include | 补 `#include` |
| `Killed` / 编译中途被杀 | OOM | 用 `--executor sequential` 或加 swap |

---

## 清理

```bash
cd ~/robot_ws
rm -rf build install log      # 全清
colcon build --symlink-install

# 或只清某个包
rm -rf build/uart_imu install/uart_imu
colcon build --packages-select uart_imu
```

---

## 验证安装产物

```bash
# 节点可执行文件存在且是 arm64 ELF
file install/uart_imu/lib/uart_imu/uart_imu_node

# launch 文件能被 ROS 找到
ros2 launch robot_bringup robot.launch.py --show-args
```
