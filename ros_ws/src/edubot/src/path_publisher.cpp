#include "path_publisher.hpp"
#include "kinematics/kinematics.hpp"

PathPublisher::PathPublisher() :
    Node("path_publisher")
{
  // Declare and acquire `target_frame` parameter
  this->declare_parameter<std::string>("frame", "ee");
  this->declare_parameter<int>("path_length", 25);

  this->_frame = this->get_parameter("frame").as_string();
  this->_path_length = this->get_parameter("path_length").as_int();

  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  _path_pub = this->create_publisher<nav_msgs::msg::Path>("ee_path", 1);
  _fk_pub = this->create_publisher<visualization_msgs::msg::Marker>("fk_ee", 1);
  _joint_sub = this->create_subscription<sensor_msgs::msg::JointState>(
                        "/joint_states", 10,
                        std::bind(&PathPublisher::_js_callback, this, std::placeholders::_1));

  using namespace std::chrono_literals;
  // Call on_timer function every second
  timer_ = this->create_wall_timer(
    100ms, std::bind(&PathPublisher::_on_timer, this));

  this->_path.header.frame_id = "world";
}

void PathPublisher::_js_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::vector<double> q = msg->position;

    Eigen::Vector3d ee_position = kinematics::fk(q.at(0), q.at(1), q.at(2), q.at(3));

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = msg->header.stamp;
    marker.header.frame_id = "world";

    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.pose.position.x = ee_position(0);
    marker.pose.position.y = ee_position(1);
    marker.pose.position.z = ee_position(2);

    marker.color.a = 0.75;
    marker.color.r = 8.0/255;
    marker.color.g = 201.0/255;
    marker.color.b = 60.0/255;

    this->_fk_pub->publish(marker);
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