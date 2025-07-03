#include "robot_sim/robot_sim.hpp"

class EdubotSim : public RobotSim
{
public:
    EdubotSim();

protected:

    void init_q() override;
    void init_names() override;

    /* Overriding because in the sim of the EduBot the
     * gripper consists of two virtual joints that need to 
     * be handled differently */
    void set_des_gripper(GripperState state) override;

    void homing() override;


private:
    const std::vector<double> HOME;
};