#include <fstream>

#include "trajectory.h"


int main(int argc, char const* argv[])
{
    auto traj = Trajectory(0.0001, 0.0);
    std::ofstream out("/tmp/traj.csv");

    auto delta = 0.001;
    for(int i=0; i<60000; ++i)
    {
        out << i*delta << "," << traj.position_at_time(i*delta) << "\n";
    }
    out << std::flush;

    return 0;
}
