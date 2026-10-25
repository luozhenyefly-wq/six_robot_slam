//动态TF
#include <geometry_msgs/msg/transform_stamped.hpp>//提供消息借口
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>//提供Quaternion类
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>//消息类型转换函数
#include <tf2_ros/transform_broadcaster.h>//坐标广播器类
#include <chrono>

using namespace rclcpp;
using namespace std;
using namespace std::chrono_literals;

class TFBroadcaster:public Node
{
private:

    shared_ptr<tf2_ros::TransformBroadcaster>broadcaster_;
    TimerBase::SharedPtr timer_;

public:

    TFBroadcaster():Node("tf_broadcaster")
    {
        this->broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_=this->create_wall_timer(100ms,std::bind(&TFBroadcaster::publish_tf,this));
    }
    void publish_tf()
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_Link";
        transform.transform.translation.x = 2.0;
        transform.transform.translation.y = 3.0;
        transform.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0,0.0,30*M_PI/180.0);
        transform.transform.rotation=tf2::toMsg(q);
        this->broadcaster_->sendTransform(transform);
    }
};

int main(int argc,char* argv[])
{
    rclcpp::init(argc,argv);
    auto node=make_shared<TFBroadcaster>();
    spin(node);
    shutdown();
}