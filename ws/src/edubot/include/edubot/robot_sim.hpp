#include "robot.hpp"

class RobotSim : public Robot
{
public:
    RobotSim();

protected:
    void set_des_q_single_rad(uint servo, float q) override;
    void set_des_q_single_deg(uint servo, float q) override;
    
    void set_des_q_rad(const std::vector<float> & q) override;
    void set_des_q_deg(const std::vector<float> & q) override;

    void set_des_gripper(GripperState state) override;
    void set_des_gripper(float o) override;

    void homing() override;

private:
    const std::vector<float> HOME;
};