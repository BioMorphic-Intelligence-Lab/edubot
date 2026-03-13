#include "robot_sim/robot_sim.hpp"

class LeRobotSim : public RobotSim
{
public:
    LeRobotSim();

protected:

    void init_q() override;
    void init_names() override;

    void homing() override;

private:
    
    // Helper function to load the home position from the parameter server
    static std::vector<double> load_home_position(rclcpp::Node& node);

    std::vector<double> HOME;
};