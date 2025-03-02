#include "lerobot_sim.hpp"

LeRobotSim::LeRobotSim(): 
    Robot(5, M_PI_2),
    HOME({DEG2RAD * 0, DEG2RAD * 60, DEG2RAD * -50,
          DEG2RAD * 50, DEG2RAD * 0})
{
    /* Init initial state and names */
    this->init_q();
    this->init_names();
 
    /* Bring to initial state */
    this->homing();
    this->set_des_gripper(GripperState::Open);
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
void LeRobotSim::set_des_q_single_rad(uint servo, float q)
{
    this->q.at(servo) = q;
}
void LeRobotSim::set_des_q_single_deg(uint servo, float q)
{
    this->set_des_q_single_rad(servo, q * RAD2DEG);
}

void LeRobotSim::set_des_q_rad(const std::vector<float> & q)
{
    assert(q.size() == this->n);
    for(uint i = 0; i < this->n; i++)
    {
        this->q.at(i) = q.at(i);
    }

}
void LeRobotSim::set_des_q_deg(const std::vector<float> & q)
{
    assert(q.size() == this->n);
    for(uint i = 0; i < this->n; i++)
    {
        this->q.at(i) = q.at(i)*DEG2RAD;
    }
}

void LeRobotSim::set_des_gripper(GripperState state)
{
    if(state == GripperState::Open)
    {
        this->gripper = std::vector<float>{(float)GripperState::Open};
    }
    else if(state == GripperState::Closed)
    {
        this->gripper = std::vector<float>{(float)GripperState::Closed};
    }
}

/* Set the currently desired gripper opening 
 * @param o: Opening degree 
 * 0 = fully closed
 * 1 = fully open
 */
void LeRobotSim::set_des_gripper(float o)
{
    /* Gripper shall be fully closed */
    if(o <= 0)
    {
        this->gripper = std::vector<float>({
             (float)GripperState::Closed
        });
    }
    /* Gripper shall be fully open */
    else if(o >= 1)
    {
        this->gripper = std::vector<float>({
             (float)GripperState::Open
        });
    }
    /* Opening somewhere in between */
    else
    {
        this->gripper = std::vector<float>({o});
    }
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