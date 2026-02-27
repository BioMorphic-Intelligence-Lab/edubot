#include "lerobot_read.hpp"
#include "feetech_cpp_lib/feetech_lib.hpp"
#include <chrono>

static const std::vector<std::string> DEFAULT_JOINT_NAMES = {
    "Shoulder_Rotation", "Shoulder_Pitch", "Elbow",
    "Wrist_Pitch", "Wrist_Roll", "Gripper"
};

LeRobotRead::LeRobotRead()
    : Node("lerobot_read")
{
    /* Parameter declaration */
    this->declare_parameter("serial_port", "/dev/ttyUSB0");
    this->declare_parameter("baud_rate", 1000000);
    this->declare_parameter("frequency", 10.0);
    this->declare_parameter("zero_positions",
        std::vector<int>({1950, 1950, 1950, 2048, 2048, 2048})
    );
    this->declare_parameter("ids",
        std::vector<int>({11, 12, 13, 14, 15, 16})
    );
    this->declare_parameter("joint_signs", 
        std::vector<double>({1.0, -1.0, -1.0, -1.0, 1.0, 1.0}));

    std::vector<long int> ids_long = this->get_parameter("ids").as_integer_array();
    std::vector<long int> zero_positions = this->get_parameter("zero_positions").as_integer_array();
    
    this->IDs.resize(ids_long.size());
    std::vector<DriverMode> operating_modes;
    for(uint8_t i = 0; i < ids_long.size(); i++)
    {
        this->IDs.at(i) = static_cast<uint8_t>(ids_long.at(i));
    }
    std::vector<double> signs = this->get_parameter("joint_signs").as_double_array();
    this->joint_signs.resize(signs.size());
    for (size_t i = 0; i < signs.size() && i < this->joint_signs.size(); i++)
        this->joint_signs[i] = signs[i];

    joint_names_ = DEFAULT_JOINT_NAMES;

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SensorDataQoS());

    RCLCPP_INFO(get_logger(), "Creating driver in read-only mode (no torque, no commands)...");
    driver_ = std::make_shared<FeetechServo>(
        this->get_parameter("serial_port").as_string(), 
        this->get_parameter("baud_rate").as_int(), 
        this->get_parameter("frequency").as_double(), 
        IDs, false, false);


    for (size_t i = 0; i < IDs.size() && i < zero_positions.size(); i++)
    {
        driver_->setHomePosition(IDs[i], static_cast<int16_t>(zero_positions[i]));
    }
    for (size_t i = 0; i < IDs.size() && i < zero_positions.size(); i++)
    {
        driver_->writeTorqueEnable(IDs[i], false);
        driver_->setOperatingModes(operating_modes);
    }
    double publish_rate = get_parameter("frequency").as_double();
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate),
        std::bind(&LeRobotRead::timer_callback, this));
    RCLCPP_INFO(get_logger(), "Publishing joint_states at %.1f Hz (passive read-only).", publish_rate);
}

void LeRobotRead::timer_callback()
{
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.header.frame_id = "";
    js.name = joint_names_;

    std::vector<double> pos = driver_->getCurrentPositions();
    std::vector<double> vel = driver_->getCurrentVelocities();

    for (size_t i = 0; i < pos.size() && i < joint_signs.size(); i++)
        pos[i] *= joint_signs[i];
    for (size_t i = 0; i < vel.size() && i < joint_signs.size(); i++)
        vel[i] *= joint_signs[i];

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
