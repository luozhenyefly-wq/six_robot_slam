# 环境配置

> 从零搭建本项目的运行环境：**ROS 2 Humble + robot_localization + slam_toolbox + rviz2**
>
> 已在 **RDK X5（aarch64 / Ubuntu 22.04）** 上完整验证。
> 在 x86 Ubuntu 22.04 上同样适用，忽略 arm64 相关说明即可。
>
> 本文只讲**装环境**。项目是做什么的见 [01_overview.md](01_overview.md)，
> 装完怎么编见 [03_build.md](03_build.md)。

---

## 1. 系统要求

| 项 | 要求 |
|---|---|
| OS | Ubuntu 22.04 (jammy) |
| ROS 2 | Humble Hawksbill |
| 架构 | aarch64（本机）或 x86_64 |
| 内存 | ≥ 2 GB（建议 4 GB；RDK X5 上约 1 GB 被 CMA/BPU/GPU 保留，`free` 只见 3 GB） |
| 磁盘 | ≥ 20 GB |
| 网络 | 能访问 ROS 镜像源（国内推荐清华源） |

---

## 2. 安装 ROS 2 Humble

### 2.1 配置 apt 源（国内推荐清华镜像）

```bash
sudo apt install -y software-properties-common curl gnupg lsb-release

# GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# 添加源（清华镜像，国内速度快）
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

> 若 `curl` 拉不到 GitHub 上的 key（网络受限），可以先用系统自带的
> `ros-archive-keyring`，或者临时用代理只拉这一个文件。

### 2.2 安装桌面版（含 rviz2、rqt）

```bash
sudo apt install -y ros-humble-desktop
```

`ros-humble-desktop` 包含：

- `rviz2` — 3D 可视化
- `rqt` 系列工具
- 常用的 `demo_nodes`、`turtlesim` 等

> 空间紧张时可只装 `ros-humble-ros-base` + `ros-humble-rviz2`，
> 但本项目推荐 desktop，省得后续补包。

### 2.3 加载环境

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

验证：

```bash
ros2 --version

# 或跑个最小例子（两个终端）
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener
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
| `robot-localization` | EKF/UKF 多传感器融合，发布 `odom → base_footprint` |
| `slam-toolbox` | 2D 激光 SLAM，发布 `map → odom` 与 `/map` |
| `nav2-bringup` | 导航栈（本项目预留，参数待补） |
| `nav2-map-server` | 提供 `map_saver_cli` 保存地图 |

编译工具：

```bash
sudo apt install -y python3-colcon-common-extensions
```

验证安装：

```bash
ros2 pkg list | grep -E "slam_toolbox|robot_localization|nav2_map_server"
```

应当输出三行。

---

## 4. 串口权限

```bash
# 加入 dialout 组，免 sudo 访问串口
sudo usermod -aG dialout $USER
# 重新登录生效（或 newgrp dialout）
```

验证：

```bash
ls -l /dev/ttyS1 /dev/ttyACM0
# crw-rw---- 1 root dialout ...   ← 组是 dialout 即可
```

---

## 5. 设备确认

```bash
# 雷达（USB 转串口）
ls -l /dev/ttyACM0
lsusb | grep -i -E "leishen|silicon|ch340|cp210"

# 下位机串口（板载 UART1）
ls -l /dev/ttyS1

# 快速验证串口能读
stty -F /dev/ttyS1 115200 raw -echo && timeout 3 cat /dev/ttyS1 | xxd | head -3
```

> **读串口必须 `stty ... raw`**，否则终端行缓冲会把二进制帧吃掉，看起来像乱码。

---

## 6. rviz2 在无 GPU 加速环境下的处理

RDK X5 的 Mali GPU 驱动对部分 OpenGL 特性支持不完整，rviz2 的 Ogre 渲染层可能
直接段错误（exit 139）。**在没有可靠 GPU 加速的板子上建议强制软件渲染：**

```bash
echo 'export LIBGL_ALWAYS_SOFTWARE=1' >> ~/.bashrc
source ~/.bashrc
```

**代价**：3D 视图会明显变慢（CPU 软件光栅化）。2D 图层（LaserScan、Map）基本不受影响。

> 若你的板子 GPU 驱动正常，可以不加这一行先试试；崩了再加。

---

## 7. 显示分辨率（远程桌面场景）

如果你通过 **VNC / 远程桌面**访问板子，且本地屏幕比例与板子输出不一致，
会出现黑边。板子默认可能只给 `1024x768`（4:3）。

查看当前：

```bash
export DISPLAY=:0
xrandr | grep -E "connected|current"
```

`0mm x 0mm` 表示 **EDID 未读到**（HDMI 热插拔时序问题），X 只能给几个旧模式。
需要**自己加 modeline**：

```bash
export DISPLAY=:0

# 例：目标 1600x1000 @60Hz（16:10）
# 像素时钟 = htotal × vtotal × 刷新率 = 2160 × 1045 × 60 ≈ 135.40 MHz
xrandr --newmode "1600x1000_60" 135.40 1600 1624 1880 2160 1000 1003 1009 1045 -hsync +vsync
xrandr --addmode HDMI-1 "1600x1000_60"
xrandr --output HDMI-1 --mode "1600x1000_60"
```

> ⚠️ **xrandr 显示的刷新率是算出来的**，不会自动凑 60。
> 公式：`刷新率 = 像素时钟 / (htotal × vtotal)`。写错会得到 48 Hz 之类奇怪值。

**开机自动设置**：写一个等 X 就绪的脚本放进 `~/.config/autostart/`，
注意 `.desktop` 的 `Exec` **不能含单引号和 `&`**，要指向 `.sh`：

```ini
[Desktop Entry]
Type=Application
Name=Keep HDMI Mode
Exec=/root/.config/autostart/keep-hdmi-display.sh
X-GNOME-Autostart-enabled=true
```

自查：`desktop-file-validate xxx.desktop`

**字体/图标偏小**（高分辨率下）：

```bash
export DISPLAY=:0
xfconf-query -c xsettings -p /Xft/DPI -n -t int -s 120          # 96 → 120 (+25%)
xfconf-query -c xsettings -p /Gtk/FontName -s "Sans 11"
xfconf-query -c xsettings -p /Gtk/MonospaceFontName -s "Monospace 11"
xfconf-query -c xfce4-desktop -p /desktop-icons/icon-size -n -t uint -s 64
```

Xfce 下 `xfconf-query` 改动**即时生效，无需注销**。

> **Xft/DPI 只影响 GTK 程序。** VS Code（Electron）要调 `window.zoomLevel`；
> rviz2/realsense-viewer（Qt）要 `QT_SCALE_FACTOR=1.25`；Firefox 要
> `about:config` 里 `layout.css.devPixelsPxPerPx = 1.25`。

---

## 8. 网络与代理

**本项目所有依赖都能从国内镜像直连装完，通常不需要代理。**

| 资源 | 镜像 | 是否需要代理 |
|---|---|---|
| ROS 2 包 | `mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu` | ❌ 不需要 |
| 系统包（aarch64） | `ports.ubuntu.com` | ❌ 不需要 |
| GitHub（源码编译时） | — | ✅ 需要 |

### 8.1 若确实需要代理

**apt 和 git 的代理配置是独立的两套**，`sudo` 也不会继承环境变量。

**apt**：

```bash
# /etc/apt/apt.conf.d/99proxy
Acquire::http::Proxy "http://<proxy-ip>:<port>";
Acquire::https::Proxy "http://<proxy-ip>:<port>";
```

**git**：

```bash
git config --global http.proxy  http://<proxy-ip>:<port>
git config --global https.proxy http://<proxy-ip>:<port>
```

**临时用**（只对当前 shell 生效）：

```bash
export http_proxy=http://<proxy-ip>:<port>
export https_proxy=$http_proxy
```

### 8.2 让板子用电脑上的代理

代理软件默认只监听 `127.0.0.1`，板子连不上。需要：

1. 代理软件里开启「**允许局域网连接 / Allow LAN**」
2. 在电脑防火墙放行该端口
3. 确认监听地址（Windows 上 `netstat -ano | findstr <port>`，
   要看到 `0.0.0.0:<port> LISTENING`，而不是 `127.0.0.1:<port>`）

验证：

```bash
curl -s -o /dev/null -w "%{http_code}\n" -x http://<电脑IP>:<端口> https://github.com
# 200 即通
```

> **注意区分「电脑 IP」和「网关 IP」。** 两者常常不同，探测错对象会一直失败。

---

## 9. 交叉检查清单

装完后逐项确认：

```bash
# 1) ROS 2 可用
ros2 pkg list | grep -E "slam_toolbox|robot_localization|nav2_map_server"

# 2) colcon 可用
colcon --help > /dev/null && echo "colcon OK"

# 3) 雷达设备
ls -l /dev/ttyACM0

# 4) 下位机串口
ls -l /dev/ttyS1

# 5) rviz2 能启动（会弹窗）
rviz2
```

全部通过即可进入 [03_build.md](03_build.md) 编译。

---

## 10. 可选：GitHub SSH 免密

```bash
ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519_github -C "user@host"
cat ~/.ssh/id_ed25519_github.pub     # 贴到 GitHub → Settings → SSH keys
```

`~/.ssh/config` 指定密钥（**没有这一步 git 不会用你的自定义密钥文件**）：

```
Host github.com
    HostName github.com
    User git
    IdentityFile ~/.ssh/id_ed25519_github
    IdentitiesOnly yes
```

验证：

```bash
ssh -T git@github.com
# Hi <username>! You've successfully authenticated...
```

---

## 11. 参考

- [ROS 2 Humble 官方安装文档](https://docs.ros.org/en/humble/Installation.html)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [robot_localization](http://docs.ros.org/en/humble/p/robot_localization/)
- [REP-105 Coordinate Frames](https://www.ros.org/reps/rep-0105.html)
