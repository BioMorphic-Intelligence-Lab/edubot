#include "lerobot_sim.hpp"

LeRobotSim::LeRobotSim(): 
    RobotSim(5, M_PI_2),
    HOME(load_home_position(*this))
{
    /* Init initial state and names */
    this->init_q();
    this->init_names();
 
    /* Bring to initial state */
    this->homing();
    this->set_des_gripper(GripperState::Closed);
}

std::vector<double> LeRobotSim::load_home_position(rclcpp::Node& node)
{
    node.declare_parameter(
        "home_position", std::vector<double>(
            {DEG2RAD * 0, DEG2RAD * 105,
            -DEG2RAD * 70, -DEG2RAD * 60, DEG2RAD * 0}
        )
    );
    return node.get_parameter("home_position").as_double_array();
}

void LeRobotSim::init_q()
{
    this->q = {0, 0, 0, 0, 0};
}

void LeRobotSim::init_names()
{
    this->names = {"Shoulder_Rotation", "Shoulder_Pitch", "Elbow",
        "Wrist_Pitch", "Wrist_Roll", "Gripper"};
}

void LeRobotSim::homing()
{
    for(uint i = 0; i < this->n; i++)
    { 
        this->q.at(i) = this->HOME.at(i);
    }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
    
  rclcpp::spin(std::make_shared<LeRobotSim>());
  rclcpp::shutdown();
  return 0;
}