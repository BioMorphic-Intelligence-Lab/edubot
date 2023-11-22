#include "ik_example.hpp"

#include <Eigen/Dense>
#include "kinematics/kinematics.hpp"

constexpr float DEG2RAD = M_PI / 180.0;

IKExample::IKExample() :
  rclcpp::Node("example_traj"),
  HOME{DEG2RAD * 45, DEG2RAD * -50, DEG2RAD * 110, DEG2RAD * 30}
{
    using namespace std::chrono_literals;

    this->_beginning = this->now();
    
    this->_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_cmds", 10);
    this->_timer = this->create_wall_timer(
      40ms, std::bind(&IKExample::_timer_callback, this));
}

void IKExample::_timer_callback()
{
  auto now = this->now();
  auto msg = trajectory_msgs::msg::JointTrajectory();
  msg.header.stamp = now;
  
  double dt = (now - this->_beginning).seconds();
  double x = 0.2 * sin(2*M_PI/10.0 * dt),
         y = 0.2 * cos(2*M_PI/10.0 * dt),
         z = 0.175;

  Eigen::Vector4d q = kinematics::ik(x, y, z);
  
  auto point = trajectory_msgs::msg::JointTrajectoryPoint();
  point.positions = {q(0), q(1), q(2), q(3)};

  msg.points = {point};

  this->_publisher->publish(msg);

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
    
  rclcpp::spin(std::make_shared<IKExample>());
  rclcpp::shutdown();
  return 0;
}