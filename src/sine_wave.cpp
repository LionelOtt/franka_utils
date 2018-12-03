#include <cmath>
#include <iostream>
#include <random>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"
#include "quintic_interpolator.h"


class SineMotionGenerator
{
    public:
        SineMotionGenerator();
};


int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }
    try
    {
        franka::Robot robot(argv[1]);
        setDefaultBehavior(robot);

        // First move the robot to a suitable joint configuration
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motion_generator(0.5, q_goal);
        std::cout << "WARNING: This example will move the robot! "
                  << "Please make sure to have the user stop button at hand!" << std::endl
                  << "Press Enter to continue..." << std::endl;
        std::cin.ignore();

        // Execute the motion to start positoin
        robot.control(motion_generator);
        std::cout << "Finished moving to initial joint configuration." << std::endl;

        // Set additional parameters always before the control loop, NEVER in the control loop!
        // Set collision behavior.
        robot.setCollisionBehavior(
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}
        );

        // Create random noise distribution
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> jitter(-0.00001, 0.00001);

        // Speed and amplitude
        auto amplitude = 0.78;
        auto speed = 1.0;

        // Test parameters
        auto every_n_steps = 10;

        // Initialize the initial position to the commanded one, to be close
        // just in case
        lula::math::SequentialQuinticInterpolator interp(
            7,
            (1.0 / 1000.0) * every_n_steps,
            5.0,
            500.0
        );

        std::array<double, 7> initial_position = q_goal;
        double time = 0.0;
        int steps = 0;
        robot.control(
            [&initial_position, &time, &gen, &jitter, &steps, &interp, amplitude, speed, every_n_steps]
            (franka::RobotState const& robot_state, franka::Duration period) -> franka::JointPositions
            {
                // Set robot position to current state upon startup
                if(time == 0.0)
                {
                    initial_position = robot_state.q_d;
                }
                time += period.toSec();

                if(steps++ % every_n_steps == 0)
                {
                    auto offset = std::sin(time * speed) * amplitude;

                    auto p = lula::math::PosVelAcc();
                    p.x = ;
                    p.xd = ;
                    p.xdd = ;
                    
                    std::string error;
                    if(!interp.AddPt(time, p, error))
                    {
                        std::cout << error << std::endl;
                    }
                }

                auto offset = std::sin(time * speed) * amplitude;
                //auto offset = std::sin(time) * 0.7 + jitter(gen);
                
                // New position with jitter on one joint
                franka::JointPositions output = {{
                    initial_position[0],
                    initial_position[1],
                    initial_position[2],
                    initial_position[3] + offset,
                    initial_position[4],
                    initial_position[5],
                    initial_position[6]
                }};

                if(time >= 3.0)
                {
                    std::cout << std::endl << "Finished motion, shutting down noise example" << std::endl;
                    return franka::MotionFinished(output);
                }

                return output;
            }
        );
    }
    catch (franka::Exception const& e)
    {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}
