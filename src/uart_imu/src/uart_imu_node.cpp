// uart_imu_node.cpp — 读取串口帧, 同时发布 里程计(/odom) 与 IMU(/imu/data)。
// 供 robot_localization 的 ekf_node 作为 odom0 + imu0 输入使用。
//
// 帧格式(8 字节, 大端, 有符号 int16 已 *100):
//     [0xAA] [yaw_hi yaw_lo] [vyaw_hi vyaw_lo] [odom_hi odom_lo] [0xBB]
//     yaw  = int16(hi,lo) / 100.0     # 原始单位: 度(deg)
//     vyaw = int16(hi,lo) / 100.0     # 原始单位: 度/s(deg/s)
//     odom = int16(hi,lo) / 100.0     # 原始单位: 厘米(cm)  -> 本帧前进距离增量
//
// 输出1: sensor_msgs/Imu      -> /imu/data   (yaw/vyaw 原样, 给 EKF 融合朝向)
// 输出2: nav_msgs/Odometry    -> /odom       (Δd 用 yaw 指方向积分成 x/y)
// 输出3: TF                   -> odom -> base_footprint (publish_tf=true 时)
//
// 重要: yaw/vyaw 原样发 /imu/data, 朝向由 EKF 融合; 这里只是拿现成的 yaw
//       指一下 Δd 的方向(这是里程计必须做的, 不是"积分陀螺仪")。
//
// 用法:
//   ros2 run uart_imu uart_imu_node --ros-args -p port:=/dev/ttyS1 -p baudrate:=115200
//   ros2 run uart_imu uart_imu_node --ros-args -p unit:=deg -p frame_id:=imu_link

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <errno.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace {

constexpr uint8_t kHead = 0xAA;
constexpr uint8_t kTail = 0xBB;
constexpr int kFrameLen = 8;      // AA + yaw(2) + vyaw(2) + odom(2) + BB
constexpr double kScale = 100.0;  // 发送端放大倍数
constexpr double kCovUnused = 1e6;  // 用不上的字段填大协方差, 告诉 EKF "别信"

// 串口波特率 -> termios 常量
speed_t baudToSpeed(int baud) {
  switch (baud) {
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 921600:  return B921600;
    default:      return B115200;
  }
}

}  // namespace

class UartImu : public rclcpp::Node {
 public:
  UartImu() : rclcpp::Node("uart_imu") {
    // ---- 参数 ----
    this->declare_parameter<std::string>("port", "/dev/ttyS1");
    this->declare_parameter<int>("baudrate", 115200);
    this->declare_parameter<std::string>("topic", "imu/data");
    this->declare_parameter<std::string>("frame_id", "imu_link");
    this->declare_parameter<std::string>("unit", "deg");  // deg | rad
    this->declare_parameter<double>("read_timeout", 0.002);  // 2ms: 每帧 yaw 都要准时间戳
    this->declare_parameter<int>("qos_depth", 20);
    this->declare_parameter<double>("orientation_cov", 0.01);//yaw协方差
    this->declare_parameter<double>("angular_velocity_cov", 0.01);
    // ---- 里程计相关 ----
    this->declare_parameter<bool>("enable_odom", true);
    this->declare_parameter<std::string>("odom_topic", "odom");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_footprint");
    this->declare_parameter<std::string>("odom_unit", "cm");  // m | cm | mm
    this->declare_parameter<bool>("publish_tf", false);  // 默认不发 TF, 由 EKF 统一发
    this->declare_parameter<bool>("debug", false);

    port_ = this->get_parameter("port").as_string();
    baudrate_ = static_cast<int>(this->get_parameter("baudrate").as_int());
    std::string topic_raw = this->get_parameter("topic").as_string();
    topic_name_ = (topic_raw.empty() || topic_raw[0] == '/') ? topic_raw : ("/" + topic_raw);
    frame_id_ = this->get_parameter("frame_id").as_string();
    unit_ = this->get_parameter("unit").as_string();
    read_timeout_ = this->get_parameter("read_timeout").as_double();
    const int depth = static_cast<int>(this->get_parameter("qos_depth").as_int());
    orientation_cov_ = this->get_parameter("orientation_cov").as_double();
    angular_velocity_cov_ = this->get_parameter("angular_velocity_cov").as_double();
    debug_ = this->get_parameter("debug").as_bool();

    enable_odom_ = this->get_parameter("enable_odom").as_bool();
    std::string odom_raw = this->get_parameter("odom_topic").as_string();
    odom_topic_ = (odom_raw.empty() || odom_raw[0] == '/') ? odom_raw : ("/" + odom_raw);
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    odom_unit_ = this->get_parameter("odom_unit").as_string();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();

    // 小写化 unit / odom_unit
    for (auto& c : unit_) c = static_cast<char>(std::tolower(c));
    for (auto& c : odom_unit_) c = static_cast<char>(std::tolower(c));

    deg2rad_ = (unit_ == "deg") ? (M_PI / 180.0) : 1.0;
    if (odom_unit_ == "cm")      odom2m_ = 0.01;
    else if (odom_unit_ == "mm") odom2m_ = 0.001;
    else                         odom2m_ = 1.0;

    // ---- 打开串口 ----
    if (!openSerial()) {
      RCLCPP_FATAL(this->get_logger(), "打开串口失败 %s", port_.c_str());
      throw std::runtime_error("open serial failed");
    }

    // ---- 发布者 (IMU 用 sensor data QoS: BEST_EFFORT) ----
    auto qos = rclcpp::QoS(rclcpp::KeepLast(depth)).best_effort();
    pub_ = this->create_publisher<sensor_msgs::msg::Imu>(topic_name_, qos);

    if (enable_odom_) {
      odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, qos);
      if (publish_tf_) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      }
    }

    x_ = 0.0;
    y_ = 0.0;

    RCLCPP_INFO(this->get_logger(),
                "[uart_imu] 串口 %s @ %d 8N1, unit=%s, frame_id=%s",
                port_.c_str(), baudrate_, unit_.c_str(), frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "[uart_imu] 发布 sensor_msgs/Imu -> %s",
                topic_name_.c_str());
    if (enable_odom_) {
      RCLCPP_INFO(this->get_logger(),
                  "[uart_imu] 发布 nav_msgs/Odometry -> %s (odom_unit=%s)",
                  odom_topic_.c_str(), odom_unit_.c_str());
      if (publish_tf_) {
        RCLCPP_INFO(this->get_logger(), "[uart_imu] 广播 TF %s -> %s",
                    odom_frame_.c_str(), base_frame_.c_str());
      }
    }

    buf_.clear();
    bad_ = 0;

    double period = read_timeout_;
    if (period < 0.01) period = 0.01;
    if (period > 0.1)  period = 0.1;
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(period),
        std::bind(&UartImu::pollSerial, this));
    RCLCPP_INFO(this->get_logger(), "uart_imu 启动完成, 等待数据...");
  }

  ~UartImu() override {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

 private:
  bool openSerial() {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_FATAL(this->get_logger(), "open(%s) 失败: %s", port_.c_str(),
                   std::strerror(errno));
      return false;
    }

    termios tty{};
    if (::tcgetattr(fd_, &tty) != 0) {
      RCLCPP_FATAL(this->get_logger(), "tcgetattr 失败: %s", std::strerror(errno));
      return false;
    }

    speed_t sp = baudToSpeed(baudrate_);
    ::cfsetispeed(&tty, sp);
    ::cfsetospeed(&tty, sp);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8 数据位
    tty.c_cflag |= (CLOCAL | CREAD);             // 忽略调制解调器线, 使能接收
    tty.c_cflag &= ~PARENB;                      // 无校验
    tty.c_cflag &= ~CSTOPB;                      // 1 停止位
    tty.c_cflag &= ~CRTSCTS;                     // 无硬件流控

    tty.c_lflag = 0;                             // raw, 无规范/回显
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL | IGNCR);
    tty.c_iflag &= ~(ISTRIP | BRKINT | PARMRK);
    tty.c_oflag &= ~OPOST;                       // 无输出处理

    // 读: 非阻塞由 O_NONBLOCK + select 控制
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_FATAL(this->get_logger(), "tcsetattr 失败: %s", std::strerror(errno));
      return false;
    }
    ::tcflush(fd_, TCIOFLUSH);
    return true;
  }

  void pollSerial() {
    if (fd_ < 0) return;

    char tmp[512];
    ssize_t n = ::read(fd_, tmp, sizeof(tmp));
    if (n > 0) {
      buf_.insert(buf_.end(), tmp, tmp + n);
      parse();
    } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      RCLCPP_ERROR(this->get_logger(), "读串口异常: %s", std::strerror(errno));
    }
  }

  void parse() {
    while (true) {
      if (static_cast<int>(buf_.size()) < kFrameLen) return;

      if (buf_[0] != kHead) {
        // 丢掉无关字节, 直到下一个 0xAA
        size_t idx = 0;
        bool found = false;
        for (size_t i = 0; i < buf_.size(); ++i) {
          if (buf_[i] == kHead) { idx = i; found = true; break; }
        }
        if (!found) {
          bad_ += static_cast<int>(buf_.size());
          buf_.clear();
          return;
        }
        bad_ += static_cast<int>(idx);
        buf_.erase(buf_.begin(), buf_.begin() + idx);
        if (static_cast<int>(buf_.size()) < kFrameLen) return;
      }

      if (buf_[kFrameLen - 1] != kTail) {
        // 帧尾不对: 这个 0xAA 是假的, 前移 1 字节重找
        bad_ += 1;
        buf_.erase(buf_.begin());
        continue;
      }

      std::vector<uint8_t> frame(buf_.begin(), buf_.begin() + kFrameLen);
      buf_.erase(buf_.begin(), buf_.begin() + kFrameLen);
      handleFrame(frame);
    }
  }

  static int16_t beInt16(uint8_t hi, uint8_t lo) {
    return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
  }

  void handleFrame(const std::vector<uint8_t>& frame) {
    int16_t yaw_raw = beInt16(frame[1], frame[2]);
    int16_t vyaw_raw = beInt16(frame[3], frame[4]);
    int16_t odom_raw = beInt16(frame[5], frame[6]);

    double yaw_deg = yaw_raw / kScale;    // 原始单位(度)
    double vyaw_deg = vyaw_raw / kScale;  // 原始单位(度/s)
    double d_cm = odom_raw / kScale;      // 原始单位(厘米), 本帧前进增量

    double yaw_rad = yaw_deg * deg2rad_;
    double vyaw_rad = vyaw_deg * deg2rad_;

    rclcpp::Time stamp = this->now();

    // ---------------- IMU 发布(原样, 给 EKF 融合朝向) ----------------
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;

    double half = yaw_rad * 0.5;
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = std::sin(half);
    msg.orientation.w = std::cos(half);

    msg.angular_velocity.x = 0.0;
    msg.angular_velocity.y = 0.0;
    msg.angular_velocity.z = vyaw_rad;

    msg.linear_acceleration.x = 0.0;
    msg.linear_acceleration.y = 0.0;
    msg.linear_acceleration.z = 0.0;

    msg.orientation_covariance[0] = orientation_cov_;
    msg.orientation_covariance[4] = orientation_cov_;
    msg.orientation_covariance[8] = kCovUnused;
    msg.angular_velocity_covariance[0] = kCovUnused;
    msg.angular_velocity_covariance[4] = kCovUnused;
    msg.angular_velocity_covariance[8] = angular_velocity_cov_;
    msg.linear_acceleration_covariance[0] = kCovUnused;
    msg.linear_acceleration_covariance[4] = kCovUnused;
    msg.linear_acceleration_covariance[8] = kCovUnused;

    pub_->publish(msg);

    // ---------------- 里程计: Δd 用现成 yaw 指方向积分 ----------------
    double d_m = d_cm * odom2m_;
    x_ += d_m * std::cos(yaw_rad);
    y_ += d_m * std::sin(yaw_rad);

    if (enable_odom_) {
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = odom_frame_;
      odom.child_frame_id = base_frame_;

      odom.pose.pose.position.x = x_;
      odom.pose.pose.position.y = y_;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation.x = 0.0;
      odom.pose.pose.orientation.y = 0.0;
      odom.pose.pose.orientation.z = std::sin(half);
      odom.pose.pose.orientation.w = std::cos(half);

      odom.twist.twist.linear.x = 0.0;
      odom.twist.twist.linear.y = 0.0;
      odom.twist.twist.linear.z = 0.0;
      odom.twist.twist.angular.x = 0.0;
      odom.twist.twist.angular.y = 0.0;
      odom.twist.twist.angular.z = vyaw_rad;

      odom.pose.covariance[0] = 0.01;         // x
      odom.pose.covariance[7] = 0.01;         // y
      odom.pose.covariance[14] = kCovUnused;  // z
      odom.pose.covariance[21] = kCovUnused;  // roll
      odom.pose.covariance[28] = kCovUnused;  // pitch
      odom.pose.covariance[35] = orientation_cov_;  // yaw
      odom.twist.covariance[0] = kCovUnused;
      odom.twist.covariance[7] = kCovUnused;
      odom.twist.covariance[14] = kCovUnused;
      odom.twist.covariance[21] = kCovUnused;
      odom.twist.covariance[28] = kCovUnused;
      odom.twist.covariance[35] = angular_velocity_cov_;

      odom_pub_->publish(odom);

      if (tf_broadcaster_) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = odom_frame_;
        t.child_frame_id = base_frame_;
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = std::sin(half);
        t.transform.rotation.w = std::cos(half);
        tf_broadcaster_->sendTransform(t);
      }
    }

    if (debug_) {
      RCLCPP_INFO(this->get_logger(),
                  "raw yaw=%d vyaw=%d odom=%d | yaw=%.4frad(%.2fdeg) "
                  "vyaw=%.4frad/s(%.2fdeg/s) d=%.4fm x=%.4f y=%.4f bad=%d",
                  yaw_raw, vyaw_raw, odom_raw, yaw_rad, yaw_deg, vyaw_rad, vyaw_deg,
                  d_m, x_, y_, bad_);
    }
  }

  // 参数
  std::string port_;
  int baudrate_{115200};
  std::string topic_name_;
  std::string frame_id_;
  std::string unit_;
  double read_timeout_{0.002};
  double orientation_cov_{0.01};
  double angular_velocity_cov_{0.01};
  bool debug_{false};
  bool enable_odom_{true};
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_unit_;
  bool publish_tf_{false};
  double deg2rad_{M_PI / 180.0};
  double odom2m_{0.01};

  // 串口
  int fd_{-1};
  std::vector<uint8_t> buf_;
  int bad_{0};

  // 累计位姿
  double x_{0.0};
  double y_{0.0};

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<UartImu>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("uart_imu"), "异常退出: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
