#include "cpp_impl/src/edubot/include/edubot/robot.hpp"
#include <unistd.h>

constexpr double DEG2RAD = M_PI/180.0;

int main(int argc, char const *argv[])
{
    std::vector<double> l = {1, 1, 1, 1};
    std::vector<double> pos = {50, 115, 180, 50}; 
    Robot robot(l, "/dev/ttyUSB0", 9600, 200);

    sleep(2);
    robot.set_des_q_deg(pos);
    sleep(2);
    robot.set_des_gripper(GripperState::Closed);
    sleep(2);
    robot.set_des_gripper(GripperState::Open);

    
    return 0;
}
