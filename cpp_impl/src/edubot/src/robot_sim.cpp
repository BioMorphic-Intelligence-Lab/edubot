#include "robot_sim.hpp"

RobotSim::RobotSim(): 
    Robot(4),
    HOME({DEG2RAD * 0, DEG2RAD * 40, DEG2RAD * 30, DEG2RAD * -30})
{
    /* Init initial state and names */
    this->init_q();
    this->init_names();
 
    /* Bring to initial state */
    this->homing();
    this->set_des_gripper(GripperState::Open);

}

void RobotSim::init_q()
{
    this->q = {0, 0, 0, 0};
}

void RobotSim::init_names()
{
    this->names = {"link1_joint", "link2_joint", "link3_joint",
        "link4_joint", "gripper_left_joint", "gripper_right_joint"};
}
void RobotSim::set_des_q_single_rad(uint servo, float q)
{
    this->q.at(servo) = q;
}
void RobotSim::set_des_q_single_deg(uint servo, float q)
{
    this->set_des_q_single_rad(servo, q * RAD2DEG);
}

void RobotSim::set_des_q_rad(const std::vector<float> & q)
{
    assert(q.size() == this->n);
    for(uint i = 0; i < this->n; i++)
    {
        this->q.at(i) = q.at(i);
    }

}
void RobotSim::set_des_q_deg(const std::vector<float> & q)
{
    assert(q.size() == this->n);
    for(uint i = 0; i < this->n; i++)
    {
        this->q.at(i) = q.at(i)*DEG2RAD;
    }
}

void RobotSim::set_des_gripper(GripperState state)
{
    if(state == GripperState::Open)
    {
        this->gripper = std::vector<float>{(float)GripperState::Open,
                                          -(float)GripperState::Open};
    }
    else if(state == GripperState::Closed)
    {
        this->gripper = std::vector<float>{(float)GripperState::Closed,
                                          -(float)GripperState::Closed};
    }
}

/* Set the currently desired gripper opening 
 * @param o: Opening degree 
 * 0 = fully closed
 * 1 = fully open
 */
void RobotSim::set_des_gripper(float o)
{
    /* Gripper shall be fully closed */
    if(o <= 0)
    {
        this->gripper = std::vector<float>({
             (float)GripperState::Closed,
            -(float)GripperState::Closed
        });
    }
    /* Gripper shall be fully open */
    else if(o >= 1)
    {
        this->gripper = std::vector<float>({
             (float)GripperState::Open,
            -(float)GripperState::Open
        });
    }
    /* Opening somewhere in between */
    else
    {
        this->gripper = std::vector<float>({o, -o});
    }
}

void RobotSim::homing()
{
    for(uint i = 0; i < this->n; i++)
    { 
        this->q.at(i) = this->HOME.at(i);
    }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
    
  rclcpp::spin(std::make_shared<RobotSim>());
  rclcpp::shutdown();
  return 0;
}