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
    const std::vector<double> HOME;
};