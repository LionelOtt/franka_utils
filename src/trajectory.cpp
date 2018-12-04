#include <cmath>
#include <iostream>

#include "trajectory.h"


Trajectory::Trajectory(double acceleration, double initial_position)
    :   m_max_acceleration(acceleration)
      , m_position(initial_position)
      , m_velocity(0.0)
      , m_acceleration(0.0)
      , m_last_time(0.0)
      , m_direction(1)
{}

Eigen::Vector3d Trajectory::get_posvelacc(double time_point)
{
    static const double phase_a_duration = 1.0;
    static const double phase_b_duration = 2.0;
    static const double total_duration = phase_a_duration + phase_b_duration;
    
    // Direction switching stuff
    static int cur_phase = 0;
    static int phase_counter = -1;
    static const std::vector<int> phase_direction = {1, -1, -1, 1};

    // Obtain time information for this cycle
    auto t_norm = time_point - static_cast<int>(time_point / total_duration)
        * total_duration;
    auto time_delta = time_point - m_last_time;
    m_last_time = time_point;

    // Hold position
    auto start_time = 0.0;
    if(start_time <= t_norm && t_norm < (start_time + phase_a_duration))
    {
        m_acceleration = 0.0;
        cur_phase = 0;
    }

    // Accelerate
    start_time += phase_a_duration;
    if(start_time <= t_norm && t_norm < (start_time + phase_b_duration))
    {
        if(cur_phase == 0)
        {
            phase_counter++;
            cur_phase = 1;
        }

        m_acceleration = phase_direction[phase_counter % phase_direction.size()] * m_max_acceleration;
    }

    m_position += m_velocity * time_delta;
    m_velocity += m_acceleration * time_delta;

    return {m_position, m_velocity, m_acceleration};
}
