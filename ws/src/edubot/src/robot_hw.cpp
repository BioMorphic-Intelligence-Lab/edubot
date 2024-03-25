#include "robot_hw.hpp"

RobotHW::RobotHW(std::string ser, int baud, int speed, int gripper_speed):
                Robot(4),
                HOME({DEG2RAD * 0, DEG2RAD * 40, DEG2RAD * 30, DEG2RAD * -30}),
                SPEED(speed),
                GRIPPER_SPEED(gripper_speed),
                MIN({500, 500, 500, 500}),
                MAX({2500, 2500, 2500, 2500}),
                RANGE({M_PI, M_PI, M_PI, M_PI})
{
    /* Open the serial port for communication */
    boost::asio::io_service io;
    this->serial = new boost::asio::serial_port(io, ser);
    this->serial->set_option(boost::asio::serial_port_base::baud_rate(baud));
    this->serial->set_option(boost::asio::serial_port_base::character_size(8 /* data bits */));
    this->serial->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    this->serial->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

    /* Bring to initial state */
    this->homing();
    this->set_des_gripper(GripperState::Open);
}

RobotHW::~RobotHW()
{
    /* Close the serial port and delete the serial port pointer */
    if (this->serial->is_open()) this->serial->close();
    delete this->serial;
}


/* Set a single servo reference position
*   @param servo: The servo index
*   @param     q: The position in radians */
void RobotHW::set_des_q_single_rad(uint servo, float q)
{
    assert(servo <= this->n);
    std::string cmd = this->format_cmd(servo,
                                       this->RAD_2_TICKS(servo, q),
                                       this->SPEED);
    cmd += "\r";

    this->write_cmd(cmd);
    this->q.at(servo) = q;
}

/* Set a single servo reference position
*   @param servo: The servo index
*   @param     q: The position in degrees */
void RobotHW::set_des_q_single_deg(uint servo, float q)
{
    this->set_des_q_single_rad(servo, q * RAD2DEG);
}

/* Set all servo reference position
*   @param     q: The position in rad */
void RobotHW::set_des_q_rad(const std::vector<float> & q)
{
    assert(q.size() == this->n);
    std::string cmd = "";
    for(uint i = 0; i < this->n; i++)
    {
        cmd += this->format_cmd(i,
          this->RAD_2_TICKS(i, q.at(i)),
          this->SPEED);
        this->q.at(i) = q.at(i);
    }
    cmd += "\r";

    this->write_cmd(cmd);
}


/* Set all servo reference position
*   @param     q: The position in degree */
void RobotHW::set_des_q_deg(const std::vector<float> & q)
{
    assert(q.size() == this->n);
    std::string cmd = "";
    for(uint i = 0; i < this->n; i++)
    {
        cmd += this->format_cmd(i,
            this->RAD_2_TICKS(i, q.at(i)*DEG2RAD),
            this->SPEED);
        this->q.at(i) = q.at(i)*DEG2RAD;
    }
    cmd += "\r";

    this->write_cmd(cmd);
}

void RobotHW::homing()
{
    std::string cmd = "";
    for(uint i = 0; i < this->n; i++)
    {
        cmd += this->format_cmd(i,
            this->RAD_2_TICKS(i, this->HOME.at(i)),
            0);
        this->q.at(i) = this->HOME.at(i);
    }
    cmd += "\r";
    
    this->write_cmd(cmd);
}

/* Set the currently desired gripper state
*    @param state: Currently desired gripper state (Open or Closed)
*/
void RobotHW::set_des_gripper(GripperState state)
{
    std::string cmd;
    if(state == GripperState::Open)
    {
        cmd = this->format_cmd(4, 900, this->GRIPPER_SPEED);
        this->gripper = GripperState::Open;
    }
    else if(state == GripperState::Closed)
    {
        cmd = this->format_cmd(4, 2500, this->GRIPPER_SPEED);
        this->gripper = GripperState::Closed;
    }
    cmd += "\r";

    this->write_cmd(cmd);
}

/* Set the currently desired gripper opening 
 * @param o: Opening degree 
 * 0 = fully closed
 * 1 = fully open
 */
void RobotHW::set_des_gripper(float o)
{
    int opened = 2200;
    int closed = 500;

    std::string cmd;

    /* Gripper shall be fully closed */
    if(o <= 0)
    {
        cmd = this->format_cmd(4, closed, this->GRIPPER_SPEED);
        this->gripper = (float)GripperState::Closed;
    }
    /* Gripper shall be fully open */
    else if(o >= 1)
    {
        cmd = this->format_cmd(4, opened, this->GRIPPER_SPEED);
        this->gripper = (float)GripperState::Open;
    }
    /* Opening somewhere in between */
    else
    {
        cmd = this->format_cmd(4,
                closed + o*(opened - closed),
                this->GRIPPER_SPEED);
        this->gripper = o;
    }
    cmd += "\r";

    this->write_cmd(cmd);
}

/* Function that uses the min, max and range to compute the 
*  equivalent radians for a given number of ticks
*   @param servo: Servo index
*   @param   rad: Angle to be transformed
*            
*  @returns number of ticks equivalent to the rad angle
*/
int RobotHW::RAD_2_TICKS(uint servo, float rad)
{
    return (this->MAX.at(servo) - this->MIN.at(servo)) * (rad / this->RANGE.at(servo)  + 0.5) 
                    + this->MIN.at(servo);
}

/* Find the correctly formatted string based on a command
*   @param servo: The servo index
*   @param   pos: The position in ticks
*   @param   vel: The velocity in ticks per second
*
*   @return correctly formatted string */
std::string RobotHW::format_cmd(uint servo, int pos, int vel)
{
    char buffer[13];
    if (vel == 0) sprintf(buffer, "#%dP%04d", servo, pos);
    else sprintf(buffer, "#%dP%04dS%03d", servo, pos, vel);
    return std::string(buffer);
}

/* Write the command to the serial port
*   @param cmd: command to be forwarded
*/
void RobotHW::write_cmd(std::string cmd)
{
    this->serial->write_some(boost::asio::buffer(cmd, cmd.length()));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
    
  rclcpp::spin(std::make_shared<RobotHW>());
  rclcpp::shutdown();
  return 0;
}