#include "mbed.h"
#include "constants.h"
#include "vector_processor.h"
#include "PID.h"


VectorProcessor::VectorProcessor(void):PID_angle(PID_A_KP, PID_A_KI, PID_A_KD, PID_A_TAU, PID_A_MIN_OUT, PID_A_MAX_OUT, PID_A_MIN_INT, PID_A_MAX_INT)
{
    prev_cumulative_angle_deg = 0.0;
    cumulative_angle_deg = 0.0;
    set_velocity = 0;
    set_angle = 0;
};

void VectorProcessor::update(float tick_count_left, float tick_count_right)
{
    // Angle calculations
    cumulative_angle_deg = (float) (360 * WHEEL_RADIUS) * (tick_count_left - tick_count_right) / (WHEEL_SEPERATION * 4 * PULSE_PER_REV);
    angle_delta = cumulative_angle_deg - prev_cumulative_angle_deg;

    // distance travelled calculation
    distance_travelled = ((float) (tick_count_left + tick_count_right) / (2 * PULSE_PER_REV * 4)) * 2 * PI * WHEEL_RADIUS;

    // calculate speed for each motor
    PID_angle.update(set_angle, cumulative_angle_deg);

    left_set_speed  = set_velocity + PID_angle.get_output();
    right_set_speed = set_velocity - PID_angle.get_output();
}

float VectorProcessor::get_cumulative_angle_deg(void)
{
    return cumulative_angle_deg;
}

float VectorProcessor::get_angle_delta(void)
{
    return angle_delta;
}

float VectorProcessor::get_left_set_speed(void)
{
    return left_set_speed;
} 

float VectorProcessor::get_right_set_speed(void)
{
    return right_set_speed;
}

void VectorProcessor::set_set_velocity(float vel)
{
    set_velocity = vel;
}

void VectorProcessor::set_set_angle(float ang)
{
    set_angle = ang;
}

float VectorProcessor::get_distance_travelled(void)
{
    return distance_travelled;
}

void VectorProcessor::reset_distance_travelled(void)
{
    distance_travelled = 0;
}

void VectorProcessor::reset_PID_angle(void)
{
    PID_angle.reset();
}
