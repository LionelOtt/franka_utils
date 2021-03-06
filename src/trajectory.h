#ifndef __TRAJECTORU_H__
#define __TRAJECTORU_H__


#include <Eigen/Core>


class Trajectory
{
    public:
        Trajectory() = default;
        Trajectory(double acceleration, double initial_position);

        Eigen::Vector3d get_posvelacc(double time_point);

    private:
        double                          m_max_acceleration;

        double                          m_position;
        double                          m_velocity;
        double                          m_acceleration;

        double                          m_last_time;
        int                             m_direction;
};


#endif /* end of include guard */
