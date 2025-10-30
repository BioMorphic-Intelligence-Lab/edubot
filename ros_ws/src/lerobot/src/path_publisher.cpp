#include "path_publisher.hpp"

PathPublisher::PathPublisher() :
    Node("path_publisher")
{
  // Declare and acquire `target_frame` parameter
  this->declare_parameter<std::string>("frame", "gripper_center");
  this->declare_parameter<int>("path_length", 25);

  this->_frame = this->get_parameter("frame").as_string();
  this->_path_length = this->get_parameter("path_length").as_int();

  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  _path_pub = this->create_publisher<nav_msgs::msg::Path>("ee_path", 1);

  using namespace std::chrono_literals;
  // Call on_timer function every second
  timer_ = this->create_wall_timer(
    100ms, std::bind(&PathPublisher::_on_timer, this));

  this->_path.header.frame_id = "world";
}

void PathPublisher::_on_timer()
{
  geometry_msgs::msg::TransformStamped t;

  // Look up for the transformation between target_frame and turtle2 frames
  // and send velocity commands for turtle2 to reach target_frame
  try {
    t = tf_buffer_->lookupTransform(
       "world", this->_frame,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      this->get_logger(), "Could not transform world to %s: %s",
        this->_frame.c_str(), ex.what());
    return;
  }
  this->_path.header.stamp = this->now();

  geometry_msgs::msg::PoseStamped p;
  p.header.stamp = t.header.stamp;
  p.header.frame_id = "world";
  p.pose.position.x = t.transform.translation.x;
  p.pose.position.y = t.transform.translation.y;
  p.pose.position.z = t.transform.translation.z;

  p.pose.orientation.w = t.transform.rotation.w;
  p.pose.orientation.x = t.transform.rotation.x;
  p.pose.orientation.y = t.transform.rotation.y;
  p.pose.orientation.z = t.transform.rotation.z;

  if(this->_poses.size() >= this->_path_length)
  {
    this->_poses.pop_front();
  }
  this->_poses.push_back(p);

  this->_path.poses = std::vector(this->_poses.begin(), this->_poses.end());

  this->_path_pub->publish(this->_path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
    
  rclcpp::spin(std::make_shared<PathPublisher>());
  rclcpp::shutdown();
  return 0;
}