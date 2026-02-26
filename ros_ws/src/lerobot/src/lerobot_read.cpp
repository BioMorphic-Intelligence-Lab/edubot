#include "lerobot_read.hpp"
#include "feetech_cpp_lib/feetech_lib.hpp"
#include <chrono>

static const std::vector<std::string> DEFAULT_JOINT_NAMES = {
    "Shoulder_Rotation", "Shoulder_Pitch", "Elbow",
    "Wrist_Pitch", "Wrist_Roll", "Gripper"
};

LeRobotRead::LeRobotRead(std::string ser, long baud, double frequency,
                         std::vector<uint8_t> ids, bool logging)
    : Node("lerobot_read"),
      ids_(std::move(ids))
{
    declare_parameter("serial_port", ser);
    declare_parameter("baud_rate", static_cast<int>(baud));
    declare_parameter("frequency", frequency);
    declare_parameter("ids", std::vector<int>(ids_.begin(), ids_.end()));
    declare_parameter("zero_positions", std::vector<int>({1950, 1950, 1950, 2048, 2048, 2048}));
    declare_parameter("joint_signs", std::vector<double>({1.0, -1.0, -1.0, -1.0, 1.0, 1.0}));
    declare_parameter("publish_rate", 50.0);
    declare_parameter("logging", logging);

    ser = get_parameter("serial_port").as_string();
    baud = static_cast<long>(get_parameter("baud_rate").as_int());
    frequency = get_parameter("frequency").as_double();
    logging = get_parameter("logging").as_bool();

    std::vector<long int> ids_long = get_parameter("ids").as_integer_array();
    ids_.clear();
    for (auto id : ids_long)
        ids_.push_back(static_cast<uint8_t>(id));

    std::vector<double> signs = get_parameter("joint_signs").as_double_array();
    joint_signs_.resize(ids_.size(), 1.0);
    for (size_t i = 0; i < signs.size() && i < joint_signs_.size(); i++)
        joint_signs_[i] = signs[i];

    joint_names_ = DEFAULT_JOINT_NAMES;

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "Creating driver in read-only mode (no torque, no commands)...");
    driver_ = std::make_shared<FeetechServo>(
        ser, baud, frequency, ids_, false, logging, true);

    std::vector<long int> zero_positions = get_parameter("zero_positions").as_integer_array();
    for (size_t i = 0; i < ids_.size() && i < zero_positions.size(); i++)
        driver_->setHomePosition(ids_[i], static_cast<int16_t>(zero_positions[i]));

    double publish_rate = get_parameter("publish_rate").as_double();
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate),
        std::bind(&LeRobotRead::timer_callback, this));
    RCLCPP_INFO(get_logger(), "Publishing joint_states at %.1f Hz (passive read-only).", publish_rate);
}

void LeRobotRead::timer_callback()
{
    driver_->readAllServoData();

    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.header.frame_id = "";
    js.name = joint_names_;

    std::vector<double> pos = driver_->getCurrentPositions();
    std::vector<double> vel = driver_->getCurrentVelocities();

    for (size_t i = 0; i < pos.size() && i < joint_signs_.size(); i++)
        pos[i] *= joint_signs_[i];
    for (size_t i = 0; i < vel.size() && i < joint_signs_.size(); i++)
        vel[i] *= joint_signs_[i];

    if (pos.size() > js.name.size())
        pos.resize(js.name.size());
    if (pos.size() < js.name.size())
        pos.resize(js.name.size(), 0.0);
    if (vel.size() != pos.size())
        vel.resize(pos.size(), 0.0);

    js.position = pos;
    js.velocity = vel;
    joint_state_pub_->publish(js);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LeRobotRead>());
    rclcpp::shutdown();
    return 0;
}
