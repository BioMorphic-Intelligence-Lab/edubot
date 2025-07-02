#include <deque>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class PathPublisher : public rclcpp::Node
{
public:
    PathPublisher();
    
private:

    void _on_timer();

    unsigned int _path_length;
    std::deque<geometry_msgs::msg::PoseStamped> _poses;
    std::string _frame;
    nav_msgs::msg::Path _path;

    rclcpp::TimerBase::SharedPtr timer_{nullptr}; 
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _path_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _fk_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_sub;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    void _js_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
   
};

