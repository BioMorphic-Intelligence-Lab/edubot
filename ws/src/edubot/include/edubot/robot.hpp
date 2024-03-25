#include <string>
#include <vector>
#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


constexpr float DEG2RAD = M_PI / 180.0;
constexpr float RAD2DEG = 180.0 / M_PI;

enum GripperState
{
    Closed = 0,
    Open = 1
};

class Robot : public rclcpp::Node
{
public:

    Robot(uint n);

    ~Robot();

protected:

    virtual void set_des_q_single_rad(uint servo, float q) = 0;
    virtual void set_des_q_single_deg(uint servo, float q) = 0;
    
    virtual void set_des_q_rad(const std::vector<float> & q) = 0;
    virtual void set_des_q_deg(const std::vector<float> & q) = 0;

    virtual void set_des_gripper(GripperState state) = 0;
    virtual void set_des_gripper(float o) = 0;

    virtual void homing() = 0;

    virtual std::vector<float> get_q();
    virtual float get_gripper();

    uint n;
    std::vector<float> q;
    float gripper;

private:
    
    
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_cmd_sub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;

    rclcpp::TimerBase::SharedPtr _timer;

    void cmd_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    void timer_callback();    

    const char *_names[6];
    const float _MAX_GRIPPER;
};