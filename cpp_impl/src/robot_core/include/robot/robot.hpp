#include <string>
#include <vector>
#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "robot_core/srv/set_mode.hpp"


constexpr float DEG2RAD = M_PI / 180.0;
constexpr float RAD2DEG = 180.0 / M_PI;

enum GripperState
{
    Closed = 0,
    Open = 1
};

enum Mode
{
    Position = 0,
    Velocity = 1
};

class Robot : public rclcpp::Node
{
public:

    Robot(uint n, float gripper_max);

    ~Robot();

protected:

    virtual void set_des_q_single_rad(uint servo, float q) = 0;
    virtual void set_des_q_single_deg(uint servo, float q) = 0;
    
    virtual void set_des_q_rad(const std::vector<float> & q) = 0;
    virtual void set_des_q_deg(const std::vector<float> & q) = 0;

    virtual void set_des_gripper(GripperState state) = 0;
    virtual void set_des_gripper(float o) = 0;

    virtual void homing() = 0;

    virtual void init_q() = 0;
    virtual void init_names() = 0;

    virtual std::vector<float> get_q();
    virtual std::vector<float> get_gripper();

    uint n;
    std::vector<float> q;
    std::vector<double> qdot_cmds;

    Mode mode;
    std::vector<std::string> names;
    std::vector<float> gripper;

private:
    
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_cmd_sub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;

    rclcpp::Service<robot_core::srv::SetMode>::SharedPtr set_mode_server;

    rclcpp::TimerBase::SharedPtr _timer;

    void cmd_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    void timer_callback(); 
    void set_mode_callback(
        const std::shared_ptr<robot_core::srv::SetMode::Request> request,
        std::shared_ptr<robot_core::srv::SetMode::Response> response
    );

    const float _MAX_GRIPPER;   
};
