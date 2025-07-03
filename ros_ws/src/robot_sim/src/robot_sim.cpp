#include <chrono>
#include "robot_sim.hpp"

RobotSim::RobotSim(uint n, double gripper_max, double sim_f)
    :Robot(n, gripper_max)
{
    this->_dt = 1.0 / sim_f;
    this->_sim_timer = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 * this->_dt)),
      std::bind(&RobotSim::timer_callback, this)
    );
}

RobotSim::~RobotSim(){};

void RobotSim::timer_callback()
{
    switch (this->mode)
    {
    case Mode::Position:
        break;
    case Mode::Velocity:
        for(uint i = 0; i< this->n; i++)
        {
            this->q.at(i) += this->qdot.at(i) * this->_dt;
        }
        break;
    default:
        break;
    }

}

void RobotSim::set_des_q_single_rad(uint servo, double q)
{
    this->q.at(servo) = q;
}
void RobotSim::set_des_qdot_single_rad(uint servo, double qdot)
{
    this->qdot.at(servo) = qdot;
}
void RobotSim::set_des_q_single_deg(uint servo, double q)
{
    this->set_des_q_single_rad(servo, q * RAD2DEG);
}
void RobotSim::set_des_qdot_single_deg(uint servo, double qdot)
{
    this->set_des_qdot_single_rad(servo, qdot * RAD2DEG);
}
void RobotSim::set_des_q_rad(const std::vector<double> & q)
{
    assert(q.size() == this->n || q.size() == this->n + 1);
    for(uint i = 0; i < this->n; i++)
    {
        this->q.at(i) = q.at(i);
    }
}
void RobotSim::set_des_qdot_rad(const std::vector<double> & qdot)
{
    assert(qdot.size() == this->n || qdot.size() == this->n + 1);
    for(uint i = 0; i < this->n; i++)
    {
        this->qdot.at(i) = qdot.at(i);
    }
}
void RobotSim::set_des_q_deg(const std::vector<double> & q)
{
    assert(q.size() == this->n || q.size() == this->n + 1);
    for(uint i = 0; i < this->n; i++)
    {
        this->q.at(i) = q.at(i)*DEG2RAD;
    }
}
void RobotSim::set_des_qdot_deg(const std::vector<double> & qdot)
{
    assert(qdot.size() == this->n || qdot.size() == this->n + 1);
    for(uint i = 0; i < this->n; i++)
    {
        this->qdot.at(i) = qdot.at(i)*DEG2RAD;
    }
}
void RobotSim::set_des_gripper(GripperState state)
{
    if(state == GripperState::Open)
    {
        this->gripper = std::vector<double>{(float)GripperState::Open};
    }
    else if(state == GripperState::Closed)
    {
        this->gripper = std::vector<double>{(float)GripperState::Closed};
    }
}

/* Set the currently desired gripper opening 
 * @param o: Opening degree 
 * 0 = fully closed
 * 1 = fully open
 */
void RobotSim::set_des_gripper(double o)
{
    /* Gripper shall be fully closed */
    if(o <= 0)
    {
        this->gripper = std::vector<double>({
             (float)GripperState::Closed
        });
    }
    /* Gripper shall be fully open */
    else if(o >= 1)
    {
        this->gripper = std::vector<double>({
             (float)GripperState::Open
        });
    }
    /* Opening somewhere in between */
    else
    {
        this->gripper = std::vector<double>({o});
    }
}
