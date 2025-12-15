#include "robot_core/robot.hpp"
#include "feetech_cpp_lib/feetech_lib.hpp"

class LeRobotHW : public Robot
{
public:
    LeRobotHW(std::string ser="/dev/ttyUSB0",
             int baud=115200,
             double frequency=50.0,
             std::vector<uint8_t> ids={11, 12, 13, 14, 15, 16},
             bool homing=false,
             bool logging=false);

protected:
    void set_des_q_single_rad(uint servo, double q) override;
    void set_des_qdot_single_rad(uint servo, double qdot) override;
    void set_des_q_single_deg(uint servo, double q) override;
    void set_des_qdot_single_deg(uint servo, double qdot) override;
    
    void set_des_q_rad(const std::vector<double> & q) override;
    void set_des_qdot_rad(const std::vector<double> & qdot) override;
    void set_des_q_deg(const std::vector<double> & q) override;
    void set_des_qdot_deg(const std::vector<double> & qdot) override;

    void set_des_gripper(GripperState state) override;
    void set_des_gripper(double o) override;
    void set_des_gripper_vel(double o) override;

    std::vector<double> get_q() override;
    std::vector<double> get_qdot() override;
    std::vector<double> get_gripper() override;

    bool set_mode(std::string mode);

    void set_mode_callback(
        const std::shared_ptr<robot_core::srv::SetMode::Request> request,
        std::shared_ptr<robot_core::srv::SetMode::Response> response
    ) override;

    void init_q() override;
    void init_names() override;

    void homing() override;

private:
    const std::vector<double> HOME;
    double gripper_open, gripper_closed;
    std::vector<uint8_t> IDs;  
    
    // Driver
    std::shared_ptr<FeetechServo> _driver;
};