// 静态TF广播器 (支持一次发布多段静态变换)
// 小滴修改 2026-10-12: 从"单段"改为"多段(列表)":
//   parents  : 每段的父坐标系名字数组
//   childs   : 每段的子坐标系名字数组
//   xyz_rpy  : 扁平数组, 每段 6 个数 [x,y,z, roll,pitch,yaw], 与 parents 一一对应
//
// 默认发布两段(本机器人):
//   1) base_footprint -> base_link     机身高度 6cm
//   2) base_link      -> laser_link    雷达安装位
//
// 用法示例:
//   ros2 run demo_cpp_tf static_tf_broadcaster
//   ros2 run demo_cpp_tf static_tf_broadcaster --ros-args \
//     -p parents:="['base_footprint']" -p childs:="['base_link']" \
//     -p xyz_rpy:="[0.0, 0.0, 0.06, 0.0, 0.0, 0.0]"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <string>
#include <vector>

using namespace rclcpp;
using namespace std;

class StaticTFBroadcaster : public Node
{
private:
    shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;

    vector<string> parents_;
    vector<string> childs_;
    vector<double> xyz_rpy_;

public:
    StaticTFBroadcaster() : Node("static_tf_broad")
    {
        // 默认值: 两段静态 TF
        parents_ = {"base_footprint", "base_link"};
        childs_  = {"base_link", "laser_link"};
        //   第1段 base_footprint->base_link : x=0, y=0, z=0.06, rpy=0
        //   第2段 base_link->laser_link     : 前方3cm, 高9.6cm, yaw=+0.257262 rad
        xyz_rpy_ = {0.0, 0.0, 0.06,   0.0, 0.0, 0.0,
                    0.03, 0.0, 0.096, 0.0, 0.0, 0.257262};

        this->declare_parameter<vector<string>>("parents", parents_);
        this->declare_parameter<vector<string>>("childs", childs_);
        this->declare_parameter<vector<double>>("xyz_rpy", xyz_rpy_);

        this->get_parameter("parents", parents_);
        this->get_parameter("childs", childs_);
        this->get_parameter("xyz_rpy", xyz_rpy_);

        if (parents_.size() != childs_.size() ||
            xyz_rpy_.size() != parents_.size() * 6)
        {
            RCLCPP_ERROR(this->get_logger(),
                "参数不匹配: parents=%zu childs=%zu xyz_rpy=%zu (应为 parents*6)",
                parents_.size(), childs_.size(), xyz_rpy_.size());
            throw std::runtime_error("static TF parameters mismatch");
        }

        this->broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        this->publish_all();
    }

    void publish_all()
    {
        for (size_t i = 0; i < parents_.size(); ++i)
        {
            double x = xyz_rpy_[i * 6 + 0];
            double y = xyz_rpy_[i * 6 + 1];
            double z = xyz_rpy_[i * 6 + 2];
            double roll = xyz_rpy_[i * 6 + 3];
            double pitch = xyz_rpy_[i * 6 + 4];
            double yaw = xyz_rpy_[i * 6 + 5];

            geometry_msgs::msg::TransformStamped transform;
            // 静态 TF 时间戳用 0: 表示任意时刻均有效。
            // 用 now() 的话, slam/下游查询激光时刻的变换会“查不到”而丢帧。
            transform.header.stamp = rclcpp::Time(0);
            transform.header.frame_id = parents_[i];
            transform.child_frame_id = childs_[i];
            transform.transform.translation.x = x;
            transform.transform.translation.y = y;
            transform.transform.translation.z = z;
            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            transform.transform.rotation = tf2::toMsg(q);
            this->broadcaster_->sendTransform(transform);

            RCLCPP_INFO(this->get_logger(),
                "[%zu] static TF %s -> %s (xyz=%.3f,%.3f,%.3f rpy=%.3f,%.3f,%.3f)",
                i, parents_[i].c_str(), childs_[i].c_str(), x, y, z, roll, pitch, yaw);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = make_shared<StaticTFBroadcaster>();
    spin(node);
    shutdown();
}
