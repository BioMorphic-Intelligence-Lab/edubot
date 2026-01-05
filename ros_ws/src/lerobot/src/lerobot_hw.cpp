#include "lerobot_hw.hpp"

LeRobotHW::LeRobotHW(std::string ser,
                     int baud,
                     double frequency,
                     std::vector<uint8_t> ids,
                     bool homing,
                     bool logging): 
    Robot(5, M_PI_2),
    HOME({DEG2RAD * 0, -DEG2RAD * 105, DEG2RAD * 70,
          DEG2RAD * 60, DEG2RAD * 0}),
    IDs(ids)
{
    this->declare_parameter("zero_positions",
        std::vector<int>({1950, 1950, 1950, 2048, 2048, 2048})
    );
    this->declare_parameter("ids",
        std::vector<int>({11, 12, 13, 14, 15, 16})
    );
    this->declare_parameter("gripper_open", M_PI_2);
    this->declare_parameter("gripper_closed", 0.0);

    this->declare_parameter("max_speed", 0.0001);

    this->gripper_open = this->get_parameter("gripper_open").as_double();
    this->gripper_closed = this->get_parameter("gripper_closed").as_double();

    this->max_speed = this->get_parameter("max_speed").as_double();

    std::vector<long int> ids_long = this->get_parameter("ids").as_integer_array();
    std::vector<long int> zero_positions = this->get_parameter("zero_positions").as_integer_array();

    for(uint8_t i = 0; i < ids_long.size(); i++)
    {
        this->IDs.at(i) = static_cast<uint8_t>(ids_long.at(i));
    }


    /* Init initial state and names */
    this->init_q();
    this->init_names();

    /* Init HW driver */
    this->_driver = std::make_shared<FeetechServo>(
        ser, baud, frequency, ids, homing, logging
    );
 
    /* Set zero positions */
    for(uint8_t i = 0; i < this->n + 1; i++)
    {
        this->_driver->setMaxSpeed(IDs.at(i), this->get_parameter("max_speed").as_double());
        this->_driver->setHomePosition(IDs.at(i), zero_positions.at(i));
    }


    /* Bring to initial state */
    this->homing();
    this->set_des_gripper(GripperState::Closed);

    /* Set the appropriate mode */
    this->set_mode(this->mode);

}

void LeRobotHW::init_q()
{
    this->q        = {0, 0, 0, 0, 0};
    this->qdot     = {0, 0, 0, 0, 0};
    this->q_des    = {0, 0, 0, 0, 0, 0};
    this->qdot_des = {0, 0, 0, 0, 0, 0};
}

void LeRobotHW::init_names()
{
    this->names = {"Shoulder_Rotation", "Shoulder_Pitch", "Elbow",
        "Wrist_Pitch", "Wrist_Roll", "Gripper"};
}
void LeRobotHW::set_des_q_single_rad(uint servo, double q)
{
    this->_driver->setReferencePosition(this->IDs.at(servo), q);
    this->q_des.at(servo) = q;
}

void LeRobotHW::set_des_qdot_single_rad(uint servo, double qdot)
{
    this->_driver->setReferenceVelocity(this->IDs.at(servo), qdot);
    this->qdot_des.at(servo) = qdot;
}
void LeRobotHW::set_des_q_single_deg(uint servo, double q)
{
    this->set_des_q_single_rad(servo, q * RAD2DEG);
}
void LeRobotHW::set_des_qdot_single_deg(uint servo, double qdot)
{
    this->set_des_qdot_single_rad(servo, qdot * RAD2DEG);
}
void LeRobotHW::set_des_q_rad(const std::vector<double> & q)
{
    assert(q.size() >= this->n);
    for(uint i = 0; i < this->n; i++)
    {
        this->set_des_q_single_rad(i, q.at(i));
    }
}
void LeRobotHW::set_des_qdot_rad(const std::vector<double> & qdot)
{    
    assert(qdot.size() >= this->n);
    for(uint i = 0; i < this->n; i++)
    {
        this->set_des_qdot_single_rad(i, qdot.at(i));
    }
}
void LeRobotHW::set_des_q_deg(const std::vector<double> & q)
{
    assert(q.size() >= this->n);
    for(uint i = 0; i < this->n; i++)
    {
        this->set_des_q_single_deg(i, q.at(i));
    }
}
void LeRobotHW::set_des_qdot_deg(const std::vector<double> & qdot)
{
    assert(qdot.size() >= this->n);
    for(uint i = 0; i < this->n; i++)
    {
        this->set_des_qdot_single_deg(i, qdot.at(i));
    }
}

void LeRobotHW::set_des_gripper(GripperState state)
{
    if(state == GripperState::Open)
    {
        this->gripper = std::vector<double>{(float)GripperState::Open};
        this->set_des_q_single_rad(this->IDs.size() - 1, this->gripper_open);
    }
    else if(state == GripperState::Closed)
    {
        this->gripper = std::vector<double>{(float)GripperState::Closed};
        this->set_des_q_single_rad(this->IDs.size() - 1, this->gripper_closed);
    }
}


/* Set the currently desired gripper opening 
 * @param o: Opening degree 
 * 0 = fully closed
 * 1 = fully open
 */
void LeRobotHW::set_des_gripper(double o)
{
    /* Gripper shall be fully closed */
    if(o <= 0)
    {
        this->set_des_gripper(GripperState::Closed);
    }
    /* Gripper shall be fully open */
    else if(o >= 1)
    {
        this->set_des_gripper(GripperState::Open);
    }
    /* Opening somewhere in between */
    else
    {
        this->gripper = std::vector<double>({o});
        this->set_des_q_single_rad(this->IDs.size() - 1, 
            gripper_closed + o * (gripper_open - gripper_closed));
    }
}

void LeRobotHW::set_des_gripper_vel(double o)
{
    this->gripper_vel.at(0) = o;
}

bool LeRobotHW::set_mode(Mode mode)
{
    bool success = false;
    if (mode == Mode::Position && this->mode != Mode::Position) {
        this->mode = Mode::Position;
        for(uint i = 0; i < this->n; i++) {
            this->_driver->setReferencePosition(this->IDs.at(i), this->q_des.at(i));
            this->_driver->setReferenceVelocity(this->IDs.at(i), 0.0);
            /* ToDo this switch breaks something. TBD*/
            this->_driver->setOperatingMode(this->IDs.at(i), DriverMode::POSITION);
            this->_driver->writeTorqueEnable(this->IDs.at(i), true);
        }
        success = true;
        RCLCPP_INFO(this->get_logger(), "Switched to Position Mode!");
    } else if (mode == Mode::Velocity) {    
        // Ensure we have the correct amount of values in the cmds vector
        while(this->qdot.size() < this->n) this->qdot.push_back(0);
        while(this->qdot_des.size() < this->n) this->qdot_des.push_back(0);

        this->mode = Mode::Velocity;
        for(uint i = 0; i < this->n; i++) {
            this->_driver->setReferencePosition(this->IDs.at(i), this->q_des.at(i));
            this->_driver->setOperatingMode(this->IDs.at(i), DriverMode::VELOCITY);
            this->_driver->setReferenceVelocity(this->IDs.at(i), this->qdot_des.at(i));
        }

        success = true;
        RCLCPP_INFO(this->get_logger(), "Switched to Velocity Mode!");
    } 
    return success;
}

void LeRobotHW::homing()
{
    for(uint i = 0; i < this->n; i++)
    {
        this->set_des_q_single_rad(i, this->HOME.at(i));
    }
}

std::vector<double> LeRobotHW::get_q()
{
    std::vector<double> q = this->_driver->getCurrentPositions();

    return q;    
}


std::vector<double> LeRobotHW::get_qdot()
{
    std::vector<double> qdot = this->_driver->getCurrentVelocities();

    return qdot;    
}

std::vector<double> LeRobotHW::get_gripper()
{
    std::vector<double> gripper = {0};

    return gripper;
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
    
  rclcpp::spin(std::make_shared<LeRobotHW>());
  rclcpp::shutdown();
  return 0;
}