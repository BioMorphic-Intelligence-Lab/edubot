#ifndef LEROBOT_READ_HPP
#define LEROBOT_READ_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <memory>
#include <vector>

class FeetechServo;

/** Node that only reads and publishes joint states; does not enable torque.
 *  Use for passive motion: move the robot by hand and read out joint positions. */
class LeRobotRead : public rclcpp::Node
{
public:
    LeRobotRead(std::string ser = "/dev/ttyUSB0",
                long baud = 1000000,
                double frequency = 25.0,
                std::vector<uint8_t> ids = {11, 12, 13, 14, 15, 16},
                bool logging = false);

private:
    void timer_callback();

    std::shared_ptr<FeetechServo> driver_;
    std::vector<uint8_t> ids_;
    std::vector<double> joint_signs_;
    std::vector<std::string> joint_names_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
