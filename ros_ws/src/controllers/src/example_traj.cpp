#include "example_traj.hpp"

constexpr double DEG2RAD = M_PI / 180.0;

ExampleTraj::ExampleTraj() :
  rclcpp::Node("example_traj")
{
    using namespace std::chrono_literals;

    // Declare all parameters
    this->declare_parameter("home",
      std::vector<double>{DEG2RAD * 0, DEG2RAD * -40,
                          DEG2RAD * 30, DEG2RAD * -30,
                          DEG2RAD * 0});
    this->home = this->get_parameter("home").as_double_array();

    this->_beginning = this->now();
    
    this->_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_cmds", 10);
    this->_timer = this->create_wall_timer(
      75ms, std::bind(&ExampleTraj::_timer_callback, this));
}

void ExampleTraj::_timer_callback()
{
  auto now = this->now();
  auto msg = trajectory_msgs::msg::JointTrajectory();
  msg.header.stamp = now;
  
  double dt = (now - this->_beginning).seconds();
  auto point = trajectory_msgs::msg::JointTrajectoryPoint();
  std::vector<double> positions;
  
  // Push joint position
  for(uint i = 0; i < this->home.size(); i++)
  {
    positions.push_back(this->home.at(i)
                        + 0.125 * M_PI * sin(2.0 * M_PI / 10.0 * dt));
  }
  // Push gripper positions
  positions.push_back(0.5 * sin(2 * M_PI / 10.0 * dt) + 0.5);

  // Finalize msg
  point.positions = positions;
  msg.points = {point};

  // Publish
  this->_publisher->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
    
  rclcpp::spin(std::make_shared<ExampleTraj>());
  rclcpp::shutdown();
  return 0;
}
