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
    /* Init initial state and names */
    this->init_q();
    this->init_names();

    /* Init HW driver */
    this->_driver = std::make_shared<FeetechServo>(
        ser, baud, frequency, ids, homing, logging
    );
 
    /* Bring to initial state */
    this->homing();
    this->set_des_gripper(GripperState::Open);
}

void LeRobotHW::init_q()
{
    this->q = {0, 0, 0, 0, 0};
}

void LeRobotHW::init_names()
{
    this->names = {"Shoulder_Rotation", "Shoulder_Pitch", "Elbow",
        "Wrist_Pitch", "Wrist_Roll", "Gripper"};
}
void LeRobotHW::set_des_q_single_rad(uint servo, double q)
{
    this->_driver->setReferencePosition(this->IDs.at(servo), q);
}

void LeRobotHW::set_des_qdot_single_rad(uint servo, double qdot)
{
    this->_driver->setReferenceVelocity(this->IDs.at(servo), qdot);
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
void LeRobotHW::set_des_gripper(double o)
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

void LeRobotHW::set_mode_callback(
    const std::shared_ptr<robot_core::srv::SetMode::Request> request,
    std::shared_ptr<robot_core::srv::SetMode::Response> response
)
{
    // Transform everything to lowercase
    std::string str = request->mode;
    for (char& c : str) {
        c = std::tolower(c);
    }

    bool success = false;
    if (str.compare("position") == 0) {
        this->mode = Mode::Position;
        for(uint i = 0; i < this->n; i++) {
            this->_driver->setOperatingMode(this->IDs.at(i), DriverMode::POSITION);
        }
        success = true;
        RCLCPP_INFO(this->get_logger(), "Switched to Position Mode!");
    } else if (str.compare("velocity") == 0) {    
        // Ensure we have the correct amount of values in the cmds vector
        while(this->qdot.size() < this->n) this->qdot.push_back(0);

        this->mode = Mode::Velocity;
        for(uint i = 0; i < this->n; i++) {
            this->_driver->setOperatingMode(this->IDs.at(i), DriverMode::VELOCITY);
        }
        success = true;
        RCLCPP_INFO(this->get_logger(), "Switched to Velocity Mode!");
    } else {
        RCLCPP_INFO(this->get_logger(), "Unknown mode '%s'!", str.c_str());
    }

    response->success = success;
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