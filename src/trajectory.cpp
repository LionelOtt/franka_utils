#include <cmath>
#include <iostream>

#include "trajectory.h"


Trajectory::Trajectory(double acceleration, double initial_position)
    :   m_position(initial_position)
      , m_velocity(0.0)
      , m_acceleration(acceleration)
      , m_last_time(0.0)
      , m_phase(0)
      , m_direction(1)
{}

Eigen::Vector3d Trajectory::get_posvelacc(double time_point)
{
    static const double phase_a_duration = 1.0;
    static const double phase_b_duration = 2.0;
    static const double phase_c_duration = 1.0;
    static const double total_duration = phase_a_duration +
        2 * phase_b_duration + phase_c_duration;
    
    // Obtain time information for this cycle
    double t_norm = time_point - static_cast<int>(time_point / total_duration)
        * total_duration;
    double t_corr = 0.0;
    double time_delta = time_point - m_last_time;
    m_last_time = time_point;
    
    double cur_acceleration = 0.0;

    // Hold position
    double start_time = 0.0;
    if(start_time <= t_norm && t_norm < (start_time + phase_a_duration))
    {
        cur_acceleration = 0.0;
        if(m_phase == 2)
        {
            m_phase = 0;
            m_direction *= -1;
        }
    }

    // Accelerate
    start_time += phase_a_duration;
    if(start_time <= t_norm && t_norm < (start_time + phase_b_duration))
    {
        t_corr = t_norm - phase_a_duration;
        cur_acceleration = m_direction * m_acceleration;
        m_phase = 1;
    }

    // Decelerate
    start_time += phase_b_duration;
    if(start_time <= t_norm && t_norm < (start_time + phase_b_duration))
    {
        t_corr = t_norm - phase_a_duration - phase_b_duration;
        cur_acceleration = -m_direction * m_acceleration;
        m_phase = 1;
    }
        
    // Hold position
    start_time += phase_b_duration;
    if(start_time <= t_norm && t_norm < (start_time + phase_c_duration))
    {
        cur_acceleration = 0.0;
        m_phase = 2;
    }

    m_position += m_velocity * t_corr;
    m_velocity += cur_acceleration * t_corr;

    return {m_position, m_velocity, cur_acceleration};
}
