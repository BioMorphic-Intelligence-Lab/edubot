#include "edubot_sim.hpp"

EdubotSim::EdubotSim(): 
    RobotSim(4, 0.02),
    HOME({DEG2RAD * 0, DEG2RAD * 40, DEG2RAD * 30, DEG2RAD * -30})
{
    /* Init initial state and names */
    this->init_q();
    this->init_names();
 
    /* Bring to initial state */
    this->homing();
    this->set_des_gripper(GripperState::Open);

}

void EdubotSim::init_q()
{
    this->q = {0, 0, 0, 0};
}

void EdubotSim::set_des_gripper(GripperState state)
{
    if(state == GripperState::Open)
    {
        this->gripper = std::vector<double>{(double)GripperState::Open,
                                          -(double)GripperState::Open};
    }
    else if(state == GripperState::Closed)
    {
        this->gripper = std::vector<double>{(double)GripperState::Closed,
                                          -(double)GripperState::Closed};
    }
}

void EdubotSim::init_names()
{
    this->names = {"link1_joint", "link2_joint", "link3_joint",
        "link4_joint", "gripper_left_joint", "gripper_right_joint"};
}
void EdubotSim::homing()
{
    for(uint i = 0; i < this->n; i++)
    { 
        this->q.at(i) = this->HOME.at(i);
    }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
    
  rclcpp::spin(std::make_shared<EdubotSim>());
  rclcpp::shutdown();
  return 0;
}