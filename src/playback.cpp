#include <cmath>
#include <iostream>
#include <fstream>
#include <random>

#include <boost/algorithm/string.hpp>
#include <boost/function.hpp>
#include <boost/lexical_cast.hpp>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"
#include "quintic_interpolator.h"


/**
 * \brief Reads the content of a file with numerical data.
 *
 * The file is assumed to have the following format:
 * v11 v12 ... v1n
 * ...
 * vn1 vn2 ... vnn
 *
 * \param fname file to read the data from
 * \param split_pattern set of characters to split the input on
 * \return data read from the file line by line
 */
template<typename T>
std::vector<Eigen::Matrix<T, Eigen::Dynamic, 1>> read(
        std::string const&              fname,
        std::string const&              split_pattern=" "
)
{
    typedef Eigen::Matrix<T, Eigen::Dynamic, 1> Vector_t;

    std::vector<Vector_t> data;
    std::ifstream input(fname);
    std::string line;

    while(input.good())
    {
        getline(input, line);

        boost::trim(line);
        std::vector<std::string> parts;
        boost::split(parts, line, boost::is_any_of(split_pattern), boost::token_compress_on);

        if(parts.size() > 1)
        {
            Vector_t values(parts.size());
            for(size_t i=0; i<parts.size(); ++i)
            {
                values[i] = boost::lexical_cast<T>(parts[i]);
            }
            data.push_back(values);
        }
    }

    return data;
}



class PlaybackMotionGenerator
{
    public:
        PlaybackMotionGenerator(std::string const& fname)
        {
            // Load data to play back on the robot
            auto data = read<double>(fname, ",");

            for(auto const& row : data)
            {
                Eigen::VectorXd q(7);
                Eigen::VectorXd dq(7);
                Eigen::VectorXd ddq(7);

                if(row.size() != 21)
                {
                    std::cout << "Bad size of data" << std::endl;
                }

                q[0] = row[0];
                q[1] = row[1];
                q[2] = row[2];
                q[3] = row[3];
                q[4] = row[4];
                q[5] = row[5];
                q[6] = row[6];

                dq[7] = row[7];
                dq[8] = row[8];
                dq[9] = row[9];
                dq[10] = row[10];
                dq[11] = row[11];
                dq[12] = row[12];
                dq[13] = row[13];

                ddq[14] = row[14];
                ddq[15] = row[15];
                ddq[16] = row[16];
                ddq[17] = row[17];
                ddq[18] = row[18];
                ddq[19] = row[19];
                ddq[20] = row[20];

                m_commands.push_back(lula::math::PosVelAcc(q, dq, ddq));
            }
        };

        franka::JointPositions operator()(
                franka::RobotState const& robot_state,
                franka::Duration        period
        )
        {
            m_time += period.toSec();

            //if(m_step_count++ % m_waypoint_distance == 0)
            //{
            //    // Feed point to the interpolator
            //    auto p = PosVelAcc(
            //        m_joint_positions[m_step_count],
            //        m_joint_velocities[m_step_count],
            //        m_joint_accelerations[m_step_count]
            //    );
            //    if(!m_interpolator.AddPt(m_time, p, m_last_error))
            //    {
            //        std::cout << m_last_error << std::endl;
            //    }
            //}
        
            //if(!interp.Eval(m_time, p, m_last_error))
            //{
            //    std::cout << m_last_error << std::endl;
            //}

            std::array<double, 7> q_command;
            Eigen::VectorXd::Map(&q_command[0], 7) = m_commands[m_step_count].x;
            //auto output = franka::JointPositions output(q_command);

            franka::JointPositions output(q_command);
            output.motion_finished = m_step_count >= m_commands.size();

            m_step_count++;

            return output;
        };

        std::array<double, 7> get_initial_pos() const
        {
            std::array<double, 7> q_command;
            Eigen::VectorXd::Map(&q_command[0], 7) = m_commands[0].x;

            return q_command;
        }

    private:
        std::string                     m_last_error;
        int                             m_step_count = 0;
        int                             m_waypoint_distance = 10;
        double                          m_time = 0;
        //lula::math::SequentialQuinticInterpolator   m_interpolator;
        std::vector<lula::math::PosVelAcc> m_commands;
};


int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>"
                  << " <playback file>" << std::endl;
        return -1;
    }
    try
    {
        // Playback motion generator
        auto playback_motion_generator = PlaybackMotionGenerator(argv[1]);

        // Initialize robot connection
        franka::Robot robot(argv[1]);
        setDefaultBehavior(robot);

        // Move robot to starting position of the playback sequence
        auto motion_generator = MotionGenerator(
                0.5,
                playback_motion_generator.get_initial_pos()
        );

        std::cout << "WARNING: Robot will move to starting position "
                     "make sure the estop is at hand." << std::endl;
        std::cout << "Press ENTER to continue" << std::endl;
        std::cin.ignore();

        robot.control(motion_generator);

        std::cout << "Finished moving to initial joint configuration." << std::endl;
        std::cout << "Starting to execute trajectory" << std::endl;


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

        robot.control(playback_motion_generator);

        std::cout << "Following trajectory completed" << std::endl;
    }
    catch (franka::Exception const& e)
    {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}
