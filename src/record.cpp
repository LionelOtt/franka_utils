#include <iostream>
#include <fstream>

#include <franka/exception.h>
#include <franka/robot.h>


int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    try {
        franka::Robot robot(argv[1]);

        std::ofstream out("/tmp/joint_data.csv");
        robot.read([&out](const franka::RobotState& robot_state)
        {
            out << robot_state.q[0] << ","
                << robot_state.q[1] << ","
                << robot_state.q[2] << ","
                << robot_state.q[3] << ","
                << robot_state.q[4] << ","
                << robot_state.q[5] << ","
                << robot_state.q[6] << ","
                << robot_state.dq[0] << ","
                << robot_state.dq[1] << ","
                << robot_state.dq[2] << ","
                << robot_state.dq[3] << ","
                << robot_state.dq[4] << ","
                << robot_state.dq[5] << ","
                << robot_state.dq[6] << ","
                << robot_state.ddq_d[0] << ","
                << robot_state.ddq_d[1] << ","
                << robot_state.ddq_d[2] << ","
                << robot_state.ddq_d[3] << ","
                << robot_state.ddq_d[4] << ","
                << robot_state.ddq_d[5] << ","
                << robot_state.ddq_d[6] << std::endl;

            return true;
        });

    } catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}
