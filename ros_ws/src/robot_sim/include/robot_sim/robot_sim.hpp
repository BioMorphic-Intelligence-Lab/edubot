#include "robot_core/robot.hpp"

class RobotSim : public Robot
{
public:
    RobotSim(uint n, double gripper_max, double sim_f=25);
    ~RobotSim();
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

    void timer_callback();
    rclcpp::TimerBase::SharedPtr _sim_timer;

private:
    double _dt;

};