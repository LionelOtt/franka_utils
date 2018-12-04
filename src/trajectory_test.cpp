#include <fstream>

#include "trajectory.h"


int main(int argc, char const* argv[])
{
    auto traj = Trajectory(0.0000001, 0.0);
    std::ofstream out("/tmp/traj.csv");

    auto delta = 0.001;
    for(int i=0; i<60000; ++i)
    {
        auto pva = traj.get_posvelacc(i*delta);
        out << i*delta << "," << pva[0] << "," << pva[1] << "," << pva[2] << "\n";
    }
    out << std::flush;

    return 0;
}
