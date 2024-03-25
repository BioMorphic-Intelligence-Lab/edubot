#include "robot.hpp"

class RobotHW : public Robot
{

public:
    RobotHW(std::string ser="/dev/ttyUSB0",
            int baud=9600,
            int speed=2000,
            int gripper_speed=9000);
    ~RobotHW();

protected:
    void set_des_q_single_rad(uint servo, float q) override;
    void set_des_q_single_deg(uint servo, float q)  override;
    
    void set_des_q_rad(const std::vector<float> & q)  override;
    void set_des_q_deg(const std::vector<float> & q)  override;

    void set_des_gripper(GripperState state)  override;
    void set_des_gripper(float o) override;

    void homing()  override;

private:

    const std::vector<float> HOME;
    const int SPEED;
    const int GRIPPER_SPEED;

    const std::vector<int> MIN;
    const std::vector<int> MAX;
    const std::vector<float> RANGE;

    boost::asio::serial_port* serial;

    void write_cmd(std::string cmd);
    std::string format_cmd(uint servo, int pos, int vel);
    int RAD_2_TICKS(uint servo, float rad);

};