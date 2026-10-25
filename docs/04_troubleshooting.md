# 常见问题排查

本项目在 RDK X5 上开发过程中真实踩过的问题,按现象归类。

---

## 一、rviz2 相关问题

### 1.1 rviz2 启动就段错误(exit 139)

**现象:**

```
qt.qpa.xcb: QXcbConnection: XCB error: 148 (Unknown), sequence: 191, ...
```

或直接 `Segmentation fault`。

**原因一:GPU 驱动跑不动**

X5 的 Mali GPU 驱动对部分 OpenGL 特性支持不完整,rviz2 的 Ogre 渲染层会崩。

**处理:**

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

写成持久化:

```bash
echo 'export LIBGL_ALWAYS_SOFTWARE=1' >> ~/.bashrc
```

**原因二:X 启动时没有 connected 输出**

如果 Xorg 启动时 HDMI 还是 disconnected,会建一个 1024x768 的兜底屏幕,
在这种屏幕下 GLX/Present 对 Ogre 不可用 → rviz2 段错误。

检查:

```bash
DISPLAY=:0 xrandr | grep -E "connected|Screen"
# 应该是 "HDMI-1 connected primary 1920x1200+0+0"
```

**处理:开机强制 HDMI 输出 + 固定分辨率**

```bash
# 1) 强制 HDMI 连接器为 on(需要在 X 启动前)
#    参考 /usr/local/bin/force-hdmi-output.sh
echo on > /sys/kernel/debug/dri/0/HDMI-A-1/force

# 2) 登录后自动设置分辨率(脚本见 keep-hdmi-display 方案)
```

### 1.2 rviz2 能开但 Map 图层不显示 / 着色器报错

**现象:**

```
active samplers with a different type refer to the same texture image unit
```

**原因:** rviz2 `indexed_8bit_image` 着色器与 Mesa 的 sampler 类型检查冲突,
属于上游 bug,**与你的数据无关**。

**绕过办法:**

- 用 `map_saver` 存图后离线查看
- 或用 `ros2 run image_view` 之类工具看 `/map` 转出的图像
- 换 `LIBGL_ALWAYS_SOFTWARE=1` 试(部分设备有效)

### 1.3 话题有数据,rviz 里却是空的

**99% 是 QoS 不匹配。**

| 话题 | 需要的 QoS |
|---|---|
| `/scan` | Reliability = **Best Effort** |
| `/map` | Durability = **Transient Local**, Reliability = Reliable |
| `/tf_static` | Durability = **Transient Local** |

在 rviz 的 Display 面板展开 Topic,把对应 QoS 改掉即可。

---

## 二、TF 与建图

### 2.1 `Message Filter dropping message: frame 'laser_link' ... earlier than all the data in the transform cache`

**原因:** slam_toolbox 要等 `laser_link` 在那个时刻的 TF,但还没到。

**处理:** 把 `slam_toolbox.yaml` 中的

```yaml
transform_timeout: 0.2   # 默认
```

改成

```yaml
transform_timeout: 1.0
```

**注意:** 这只是"多等一会儿",不影响精度。

### 2.2 `Message Filter dropping ... queue is full`

**这是低算力设备上"处理不过来,丢旧帧"**,**不影响地图质量**。

判断方法:

```bash
# 看 TF 有没有断档
ros2 run tf2_ros tf2_echo odom base_footprint
# dt 应该在 0.04~0.06 s(20 Hz)左右,不应出现 >0.3 s 的跳变
```

如果 TF 连续、`/scan` 也连续,只是 slam 报 queue full,**可以忽略**。

要缓解:

- 提高 `minimum_time_interval`
- 降低雷达转速
- 换更快的硬件

### 2.3 地图不更新

排查顺序:

```bash
# 1) slam 是否在处理扫描
ros2 topic echo /map --once        # 能出数据说明在发
# 2) TF 链是否完整
ros2 run tf2_ros tf2_echo map base_footprint
# 3) rviz 的 Map 图层 QoS 是否是 Transient Local
```

### 2.4 回原点不重合 / 地图重影

**这是里程计漂移**,不是 slam 的问题。见下一节。

---

## 三、里程计漂移

### 3.1 定位方法

**做三项测量,分别定位不同误差源:**

| 测量 | 方法 | 判断 |
|---|---|---|
| **陀螺零偏** | 静止 30 秒,看 yaw 变化 | 漂 > 2° → 零偏未补偿 |
| **尺度误差** | 直推 1 米,看 `/odom` 的 x | 明显 ≠ 1.0 → 换算系数错 |
| **角度误差** | 慢转 90°,看 yaw 增量 | ≠ 90° → 角度换算错 |

命令:

```bash
ros2 topic echo /odom --field pose.pose.position
ros2 topic echo /imu/data --field orientation
```

### 3.2 常见原因

| 现象 | 原因 |
|---|---|
| 静止时 yaw 缓慢增长 | 陀螺零偏,需上电静止标定 |
| 推动 1 m 显示 0.8 m | 里程换算系数(单位 cm→m)或轮径参数错 |
| 转向角度偏大/偏小 | 角度换算,或坐标系手性搞反 |

**坐标系提醒:** 下位机底层可能用"顺时针为正",ROS 必须"逆时针为正"(REP-103),
两层之间一定要做符号转换。

---

## 四、串口与传感器

### 4.1 `Permission denied` 打不开串口

```bash
sudo usermod -aG dialout $USER
# 重新登录
```

临时验证:

```bash
sudo chmod 666 /dev/ttyS1   # 不推荐长期使用
```

### 4.2 串口收到乱码

**终端读串口必须 set raw:**

```bash
stty -F /dev/ttyS1 115200 raw -echo
cat /dev/ttyS1 | xxd
```

不做 `raw` 的话,终端行缓冲会把二进制帧吃掉。

### 4.3 雷达打不开 `/dev/ttyACM0`

```bash
# 确认设备在
lsusb | grep -i lexis  # 或对应厂商
dmesg | tail -20       # 看是否有 USB 枚举日志

# 确认没被别的进程占用
fuser /dev/ttyACM0
```

雷达波特率是 **400000**(非标准值),普通串口工具可能打不开。

---

## 五、编译

### 5.1 编译被 `Killed`

**OOM。** X5 只有 3 GB 可用内存:

```bash
colcon build --symlink-install --executor sequential
```

或加 swap:

```bash
sudo fallocate -l 2G /swapfile && sudo chmod 600 /swapfile
sudo mkswap /swapfile && sudo swapon /swapfile
```

### 5.2 `undefined reference to Xxx::Xxx()`

**新增的 `.cpp` 没加进 `CMakeLists.txt` 的 `add_executable()`。**

ament/colcon **不会自动收集源文件**,每个新文件都要手动加。

### 5.3 编译成功但运行时找不到节点

```bash
# 检查产物
file install/uart_imu/lib/uart_imu/uart_imu_node
# 应为: ELF 64-bit LSB pie executable, ARM aarch64

# 检查环境
echo $AMENT_PREFIX_PATH
# 应包含 ~/robot_ws/install
```

---

## 六、RDK X5 平台特有

### 6.1 `free -h` 只看到 3 GB,但硬件是 4 GB

**正常。** 约 1 GB 被 SoC 的 CMA(视频)/BPU/GPU 保留,系统不可见。

### 6.2 时钟显示 2000 年

**板子 RTC 不可靠。** 影响:

- `ros2 bag record` 的文件名
- 日志时间戳

不影响 TF 和 SLAM(它们用相对时间)。

### 6.3 VNC 远程桌面黑边

**原因:** 板子输出分辨率比例与本地屏幕不一致。

**处理:** 用 `xrandr` 自建 modeline 匹配比例:

```bash
export DISPLAY=:0
# 例:目标 1600x1000 (16:10,与 3200x2000 屏幕一致)
# 像素时钟 = htotal × vtotal × 刷新率 = 2160 × 1045 × 60 ≈ 135.40 MHz
xrandr --newmode "1600x1000_60" 135.40 1600 1624 1880 2160 1000 1003 1009 1045 -hsync +vsync
xrandr --addmode HDMI-1 "1600x1000_60"
xrandr --output HDMI-1 --mode "1600x1000_60"
```

> **注意:`xrandr` 显示的模式刷新率是算出来的。**
> `pixel_clock / (htotal × vtotal)` 才是真实刷新率,不会自动帮你凑 60 Hz。

---

## 七、其他踩坑

### 7.1 `.desktop` 自启动脚本不生效

**`Exec` 里不能有单引号 `'` 和 `&`** —— 这是 desktop 规范的保留字符,
一旦出现 XFCE 会**静默忽略整个文件**。

**正确做法:** `Exec` 只指向一个 `.sh` 脚本:

```ini
Exec=/path/to/launcher.sh
```

自查:

```bash
desktop-file-validate xxx.desktop
```

### 7.2 `pkill -f` 把自己杀掉了

`pkill -f <pattern>` 会匹配到**执行这条命令的那个 shell**(因为命令行里含 pattern)。

**正确做法:**

```bash
pgrep -af "node_name"        # 先看 PID
kill -9 <具体PID>
```

### 7.3 后台任务被回收

```bash
setsid bash -c 'long_command > /tmp/log 2>&1' &
```

`nohup` 在某些会话管理下不够,`setsid` 才能真脱离。

### 7.4 `sh` 里 `ldconfig` 找不到

`/sbin` 不在非登录 shell 的 PATH 里。用完整路径:

```bash
/sbin/ldconfig
```
