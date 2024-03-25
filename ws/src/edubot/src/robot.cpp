#include "robot.hpp"
#include <iostream>

#include <cmath>
#include <cassert>

Robot::Robot(uint n):
                Node("robot"),
                n(n),
                q({0.0, 0.0, 0.0, 0.0}),
                gripper((float)GripperState::Closed),
                _names{"link1_joint", "link2_joint", "link3_joint", "link4_joint", "gripper_left_joint", "gripper_right_joint"},
                _MAX_GRIPPER{0.02}
{
    using namespace std::chrono_literals;

    this->declare_parameter("f", 24.0);
    this->declare_parameter("pub_topic", "joint_states");
    this->declare_parameter("sub_topic", "joint_cmds");

    this->joint_cmd_sub = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        this->get_parameter("sub_topic").as_string(),
        10,
        std::bind(&Robot::cmd_callback,
                this,
                std::placeholders::_1)
    );

    this->joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>(
        this->get_parameter("pub_topic").as_string(), 10);

    this->_timer = this->create_wall_timer(1.0 / this->get_parameter("f").as_double() * 1s,
                                           std::bind(&Robot::timer_callback,
                                                     this));


}

Robot::~Robot()
{
}

/* Callback function for when a new desired trajectory is published. 
 * We assume the trajectory only contains one JointTrajectoryPoint: the desired one  */
void Robot::cmd_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    std::vector<double> positions = msg->points[0].positions;
    std::vector<float> des_q(this->n);

    for(uint i = 0; i < this->n; i++)
    {
        des_q.at(i) = positions.at(i);
    } 
    this->set_des_q_rad(des_q);

    /* If this trajectory setpoint also contains gripper commands */
    if(positions.size() == this->n + 1)
        this->set_des_gripper(positions.at(n));
}
    
void Robot::timer_callback()
{
  sensor_msgs::msg::JointState js;
  js.name = {"", "", "", "", "", ""};
  js.header.stamp = this->now();

  std::vector<float> q_float = this->get_q();
  // Get gripper state scaled by its max opening value
  float gripper = this->get_gripper();
  // We have to append it twice to since left gripper and right 
  // Gripper are treated independently by rviz
  q_float.push_back(this->_MAX_GRIPPER * gripper);
  q_float.push_back(- this->_MAX_GRIPPER * gripper);

  std::vector<double> q(q_float.begin(), q_float.end());

  for(uint i = 0; i < 6; i++) js.name[i] = this->_names[i];
  
  js.position = q;

  this->joint_state_pub->publish(js);

}    

std::vector<float> Robot::get_q()
{
    return this->q;
}

float Robot::get_gripper()
{
    return this->gripper;
}

