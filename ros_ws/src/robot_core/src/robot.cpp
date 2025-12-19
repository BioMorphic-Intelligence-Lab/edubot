#include "robot.hpp"
#include <iostream>

#include <cmath>
#include <cassert>

Robot::Robot(uint n, double max_gripper):
                Node("robot"),
                n(n),
                gripper((float)GripperState::Closed),
                _MAX_GRIPPER(max_gripper)
{
    using namespace std::chrono_literals;

    this->declare_parameter("f", 100.0);
    this->declare_parameter("pub_topic", "joint_states");
    this->declare_parameter("sub_topic", "joint_cmds");
    this->declare_parameter("vel_sub_topic", "joint_vel_cmds");    
    this->declare_parameter("mode", "position");

    if (this->get_parameter("mode").as_string().compare("position") == 0) {
      this->mode = Mode::Position;
    } else if (this->get_parameter("mode").as_string().compare("velocity") == 0) {
      this->mode = Mode::Velocity;
    }

    while(this->qdot.size() < this->n) this->qdot.push_back(0);
    while(this->gripper_vel.size() < 1) this->gripper_vel.push_back(0);

    this->joint_cmd_sub = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        this->get_parameter("sub_topic").as_string(),
        10,
        std::bind(&Robot::cmd_callback,
                this,
                std::placeholders::_1)
    ); 

    this->joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>(
        this->get_parameter("pub_topic").as_string(), 10);

    this->set_mode_server = this->create_service<robot_core::srv::SetMode>("set_mode",
      std::bind(&Robot::set_mode_callback, this, 
                std::placeholders::_1, std::placeholders::_2)  
    );

    this->_timer = this->create_wall_timer(1.0 / this->get_parameter("f").as_double() * 1s,
                                           std::bind(&Robot::timer_callback,
                                                     this));
}

Robot::~Robot()
{
}

void Robot::set_mode_callback(
    const std::shared_ptr<robot_core::srv::SetMode::Request> request,
    std::shared_ptr<robot_core::srv::SetMode::Response> response
) {
  // Transform everything to lowercase
  std::string str = request->mode;
  for (char& c : str) {
    c = std::tolower(c);
  }

  bool success = false;
  if (str.compare("position") == 0) {
    this->mode = Mode::Position;
    success = true;
    RCLCPP_INFO(this->get_logger(), "Switched to Position Mode!");
  } else if (str.compare("velocity") == 0) {    
    // Ensure we have the correct amount of values in the cmds vector
    while(this->qdot.size() < this->n) this->qdot.push_back(0);

    this->mode = Mode::Velocity;
    success = true;
    RCLCPP_INFO(this->get_logger(), "Switched to Velocity Mode!");
  } else {
    RCLCPP_INFO(this->get_logger(), "Unknown mode '%s'!", str.c_str());
  }

  response->success = success;
}

/* Callback function for when a new desired reference is published. 
 * We assume the trajectory only contains one JointTrajectoryPoint: the desired one */
void Robot::cmd_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    switch(this->mode){
      case Mode::Position:
      {
        std::vector<double> positions = msg->points[0].positions;
        std::vector<double> des_q(this->n);
        if(positions.size() < this->n){
          std::cout << "Joint Trajectory Command rejected. Too few inputs" << std::endl;
          return;
        }

        for(uint i = 0; i < this->n; i++)
        {
            des_q.at(i) = positions.at(i);
        } 
        this->set_des_q_rad(des_q);

        /* If this trajectory setpoint also contains gripper commands */
        if(positions.size() == this->n + 1)
            this->set_des_gripper(positions.at(n));

        break;
      }
      case Mode::Velocity:
      {
        std::vector<double> velocities = msg->points[0].velocities;
        if(velocities.size() < this->n){
          std::cout << "Joint Velocity Command rejected. Too few inputs" << std::endl;
          return;
        }

        this->set_des_qdot_rad(velocities);
        /* If this trajectory setpoint also contains gripper commands */
        if(velocities.size() == this->n + 1)
            this->set_des_gripper_vel(velocities.at(n));
        break;
      }
    }
}
 
void Robot::timer_callback()
{
  sensor_msgs::msg::JointState js;
  js.name = this->names;
  js.header.stamp = this->now();

  // Get Joint states
  std::vector<double> q_pub = this->get_q();
  std::vector<double> qdot_pub = this->get_qdot();

  // Get gripper state scaled by its max opening value
  std::vector<double> gripper = this->get_gripper();
  std::vector<double> gripper_vel = this->get_gripper_vel();

  // We may have to append it twice since left gripper and right 
  // Gripper are treated independently by rviz
  for(double val : this->gripper)
    q_pub.push_back(this->_MAX_GRIPPER * val);

  // Publish current joint state
  js.position = q_pub;
  js.velocity = qdot_pub;
  this->joint_state_pub->publish(js);
}    

std::vector<double> Robot::get_q()
{
  return this->q;
}

std::vector<double> Robot::get_qdot()
{
  /* By default we assume we have no velocity feedback
   * and simply return zero */
  return this->qdot;
}

std::vector<double> Robot::get_gripper()
{
    return this->gripper;
}

std::vector<double> Robot::get_gripper_vel()
{
    return this->gripper_vel;
}



