#include "example_pos_traj.hpp"
#include <cmath>

constexpr double DEG2RAD = M_PI / 180.0;

ExampleTraj::ExampleTraj() :
  rclcpp::Node("example_traj")
{
    using namespace std::chrono_literals;

    // Declare all parameters (home matches URDF/sim: 0, 105°, -70°, -60°, 0 deg)
    this->declare_parameter("home",
      std::vector<double>{DEG2RAD * 0, DEG2RAD * 105,
                          -DEG2RAD * 70, -DEG2RAD * 60,
                          DEG2RAD * 0});
    this->home = this->get_parameter("home").as_double_array();

    this->_beginning = this->now();
    
    // Make QoS for publisher
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    qos.durability_volatile();

    this->_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_cmds", qos);
    // 0.04 s period to match example_pos_traj.py
    this->_timer = this->create_wall_timer(
      std::chrono::duration<double>(0.04), std::bind(&ExampleTraj::_timer_callback, this));
}

void ExampleTraj::_timer_callback()
{
  auto now = this->now();
  auto msg = trajectory_msgs::msg::JointTrajectory();
  msg.header.stamp = now;

  double dt = (now - this->_beginning).seconds();
  const double s = 0.125 * M_PI * std::sin(2.0 * M_PI / 10.0 * dt);
  const double g = 0.5 * std::sin(2.0 * M_PI / 10.0 * dt) + 0.5;

  // Same formula as example_pos_traj.py
  auto point = trajectory_msgs::msg::JointTrajectoryPoint();
  point.positions = {
    this->home.at(0) + s,
    this->home.at(1) - 0.25 * M_PI + s,
    this->home.at(2) + 0.125 * M_PI + s,
    this->home.at(3) + 0.25 * M_PI + s,
    this->home.at(4) + s,
    g
  };
  msg.points = {point};

  this->_publisher->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
    
  rclcpp::spin(std::make_shared<ExampleTraj>());
  rclcpp::shutdown();
  return 0;
}
