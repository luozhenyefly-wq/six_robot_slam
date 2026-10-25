#include <geometry_msgs/msg/transform_stamped.hpp>//提供消息借口
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>//提供Quaternion类
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>//消息类型转换函数
#include <tf2_ros/transform_listener.h>//坐标监听类
#include <chrono>
#include <tf2_ros/buffer.h>//提供buffer
#include <tf2/utils.h>

using namespace rclcpp;
using namespace std;
using namespace std::chrono_literals;

class TFListener:public Node
{
private:

    shared_ptr<tf2_ros::TransformListener>listener_;
    TimerBase::SharedPtr timer_;
    shared_ptr<tf2_ros::Buffer>buffer_;

public:

    TFListener():Node("tf_listener")
    {
        this->buffer_=std::make_shared<tf2_ros::Buffer>(this->get_clock());
        this->listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_,this);
        timer_=this->create_wall_timer(1s,std::bind(&TFListener::getTransform,this));     
    }

    void getTransform()
    {
        //查询坐标关系
        try
        {
            //查询坐标关系
            const auto transform = buffer_->lookupTransform("base_Link","target_point",
                                this->get_clock()->now(),Duration::from_seconds(1.0f)); 
            //获取查询结果
            auto translation = transform.transform.translation;
            auto rotation = transform.transform.rotation;
            double y,p,r;
            tf2::getEulerYPR(rotation,y,p,r);
            RCLCPP_INFO(get_logger(),"平移：%f %f %f",translation.x,translation.y,translation.z);
            RCLCPP_INFO(get_logger(),"旋转：%f %f %f",y,p,r);
        }
        catch(const std::exception&e)
        {
            RCLCPP_WARN(get_logger(),"%s",e.what());
        }
    }
};

int main(int argc,char* argv[])
{
    rclcpp::init(argc,argv);
    auto node=make_shared<TFListener>();
    spin(node);
    shutdown();
}