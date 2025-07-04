#include <chrono>
#include "edubot_hw.hpp"

EdubotHW::EdubotHW(std::string ser, int baud, int speed, int gripper_speed):
                Robot(4, 0.02),
                HOME({DEG2RAD * 0, DEG2RAD * 40, DEG2RAD * 30, DEG2RAD * -30}),
                SPEED(speed),
                GRIPPER_SPEED(gripper_speed),
                DT(0.01),
                MIN({500, 500, 500, 500}),
                MAX({2500, 2500, 2500, 2500}),
                RANGE({M_PI, M_PI, M_PI, M_PI})
{
    /* Init initial state and names */
    this->init_q();
    this->init_names();
    
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

    /* Init timer for intergrating velocity in velocity mode */
    this->_vel_integrator_timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 * DT)),
        std::bind(&EdubotHW::vel_int_callback, this)
    );
}

EdubotHW::~EdubotHW()
{
    /* Close the serial port and delete the serial port pointer */
    if (this->serial->is_open()) this->serial->close();
    delete this->serial;
}

void EdubotHW::vel_int_callback()
{
    switch (this->mode)
    {
    case Mode::Position:
        break;
    case Mode::Velocity:
    {
        std::vector<double> new_q;
        for(uint i = 0; i < this->n; i++)
        {
            new_q.push_back(this->q.at(i) + this->qdot.at(i) * DT);
        }
        this->set_des_q_rad(new_q);
        break;
    }
    default:
        break;
    }
}

void EdubotHW::init_q()
{
    this->q = {0, 0, 0, 0};
}

void EdubotHW::init_names()
{
    this->names = {"link1_joint", "link2_joint", "link3_joint",
        "link4_joint", "gripper_joint"};
}


/* Set a single servo reference position
*   @param servo: The servo index
*   @param     q: The position in radians */
void EdubotHW::set_des_q_single_rad(uint servo, double q)
{
    assert(servo <= this->n);
    std::string cmd = this->format_cmd(servo,
                                       this->RAD_2_TICKS(servo, q),
                                       this->SPEED);
    cmd += "\r";

    this->write_cmd(cmd);
    this->q.at(servo) = q;
}
void EdubotHW::set_des_qdot_single_rad(uint servo, double qdot)
{
    this->qdot.at(servo) = qdot;
}
/* Set a single servo reference position
*   @param servo: The servo index
*   @param     q: The position in degrees */
void EdubotHW::set_des_q_single_deg(uint servo, double q)
{
    this->set_des_q_single_rad(servo, q * RAD2DEG);
}
void EdubotHW::set_des_qdot_single_deg(uint servo, double qdot)
{
    this->set_des_qdot_single_rad(servo, qdot * RAD2DEG);
    
}
/* Set all servo reference position
*   @param     q: The position in rad */
void EdubotHW::set_des_q_rad(const std::vector<double> & q)
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
void EdubotHW::set_des_qdot_rad(const std::vector<double> & qdot)
{
    assert(qdot.size() == this->n);
    this->qdot = qdot;
}


/* Set all servo reference position
*   @param     q: The position in degree */
void EdubotHW::set_des_q_deg(const std::vector<double> & q)
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

void EdubotHW::set_des_qdot_deg(const std::vector<double> & qdot)
{
    assert(qdot.size() == this->n);
    for(uint i=0; i < this->n; i++)
    {
        this->qdot.at(i) = qdot.at(i) * DEG2RAD;
    }
}
void EdubotHW::homing()
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
void EdubotHW::set_des_gripper(GripperState state)
{
    std::string cmd;
    if(state == GripperState::Open)
    {
        cmd = this->format_cmd(4, 900, this->GRIPPER_SPEED);
        this->gripper = std::vector<double>{GripperState::Open};
    }
    else if(state == GripperState::Closed)
    {
        cmd = this->format_cmd(4, 2500, this->GRIPPER_SPEED);
        this->gripper = std::vector<double>{GripperState::Closed};
    }
    cmd += "\r";

    this->write_cmd(cmd);
}

/* Set the currently desired gripper opening 
 * @param o: Opening degree 
 * 0 = fully closed
 * 1 = fully open
 */
void EdubotHW::set_des_gripper(double o)
{
    int opened = 2200;
    int closed = 500;

    std::string cmd;

    /* Gripper shall be fully closed */
    if(o <= 0)
    {
        cmd = this->format_cmd(4, closed, this->GRIPPER_SPEED);
        this->gripper = std::vector<double>{(float)GripperState::Closed};
    }
    /* Gripper shall be fully open */
    else if(o >= 1)
    {
        cmd = this->format_cmd(4, opened, this->GRIPPER_SPEED);
        this->gripper = std::vector<double>{(float)GripperState::Open};
    }
    /* Opening somewhere in between */
    else
    {
        cmd = this->format_cmd(4,
                closed + o*(opened - closed),
                this->GRIPPER_SPEED);
        this->gripper = std::vector<double>{o};
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
int EdubotHW::RAD_2_TICKS(uint servo, double rad)
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
std::string EdubotHW::format_cmd(uint servo, int pos, int vel)
{
    char buffer[13];
    if (vel == 0) sprintf(buffer, "#%dP%04d", servo, pos);
    else sprintf(buffer, "#%dP%04dS%03d", servo, pos, vel);
    return std::string(buffer);
}

/* Write the command to the serial port
*   @param cmd: command to be forwarded
*/
void EdubotHW::write_cmd(std::string cmd)
{
    this->serial->write_some(boost::asio::buffer(cmd, cmd.length()));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
    
  rclcpp::spin(std::make_shared<EdubotHW>());
  rclcpp::shutdown();
  return 0;
}