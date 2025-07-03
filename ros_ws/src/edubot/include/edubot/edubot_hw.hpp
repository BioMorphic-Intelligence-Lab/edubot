#include "robot/robot.hpp"

class EdubotHW : public Robot
{

public:
    EdubotHW(std::string ser="/dev/ttyUSB0",
            int baud=9600,
            int speed=1000,
            int gripper_speed=9000);
    ~EdubotHW();

protected:
    void set_des_q_single_rad(uint servo, float q) override;
    void set_des_q_single_deg(uint servo, float q)  override;
    
    void set_des_q_rad(const std::vector<float> & q)  override;
    void set_des_q_deg(const std::vector<float> & q)  override;

    void set_des_gripper(GripperState state)  override;
    void set_des_gripper(float o) override;

    void init_q() override;
    void init_names() override;

    void homing()  override;

    void set_mode_callback(
        const std::shared_ptr<robot_core::srv::SetMode::Request> request,
        std::shared_ptr<robot_core::srv::SetMode::Response> response
    ) override;


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