#include "robot_core/robot.hpp"
#include "feetech_cpp_lib/feetech_lib.hpp"

class LeRobotHW : public Robot
{
public:
    LeRobotHW();

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

    bool set_mode(Mode mode);

    void init_q() override;
    void init_names() override;

    void homing() override;

private:
    const std::vector<double> HOME;
    double gripper_open, gripper_closed, max_speed;
    std::vector<uint8_t> IDs;
    /** Per-joint sign correction so HW matches URDF/sim convention (1 or -1 per joint). */
    std::vector<double> joint_signs;

    // Driver
    std::shared_ptr<FeetechServo> _driver;
};