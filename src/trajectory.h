#ifndef __TRAJECTORU_H__
#define __TRAJECTORU_H__


class Trajectory
{
    public:
        Trajectory(double acceleration, double initial_position);

        double position_at_time(double time_point);

    private:
        double                          m_position;
        double                          m_velocity;
        double                          m_acceleration;
        double                          m_last_time;
        int                             m_phase;
        int                             m_direction;
};


#endif /* end of include guard */
