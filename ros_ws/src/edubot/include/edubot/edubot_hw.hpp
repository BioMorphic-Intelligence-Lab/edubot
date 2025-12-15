#include "robot_core/robot.hpp"

class EdubotHW : public Robot
{

public:
    EdubotHW(std::string ser="/dev/ttyUSB0",
            int baud=9600,
            int speed=1000,
            int gripper_speed=9000);
    ~EdubotHW();

protected:
    void set_des_q_single_rad(uint servo, double q) override;
    void set_des_qdot_single_rad(uint servo, double qdot) override;
    void set_des_q_single_deg(uint servo, double q) override;
    void set_des_qdot_single_deg(uint servo, double qdot) override;
    
    void set_des_q_rad(const std::vector<double> & q) override;
    void set_des_qdot_rad(const std::vector<double> & qdot) override;
    void set_des_q_deg(const std::vector<double> & q) override;
    void set_des_qdot_deg(const std::vector<double> & qdot) override;

    void set_des_gripper(GripperState state)  override;
    void set_des_gripper(double o) override;
    void set_des_gripper_vel(double o) override;

    void init_q() override;
    void init_names() override;

    void homing()  override;

private:

    const std::vector<double> HOME;
    const int SPEED;
    const int GRIPPER_SPEED;
    const double DT;

    const std::vector<int> MIN;
    const std::vector<int> MAX;
    const std::vector<double> RANGE;

    boost::asio::serial_port* serial;

    void vel_int_callback();
    void write_cmd(std::string cmd);
    std::string format_cmd(uint servo, int pos, int vel);
    int RAD_2_TICKS(uint servo, double rad);

    rclcpp::TimerBase::SharedPtr _vel_integrator_timer;

};