#include "mbed.h"
#include "constants.h"
#include "vector_processor.h"


VectorProcessor::VectorProcessor(void) 
{
    prev_cumulative_angle_deg = 0.0;
    cumulative_angle_deg = 0.0;
    set_velocity = 0;
    set_angle = 0;
};

void VectorProcessor::update(float tick_count_left, float tick_count_right)
{
    cumulative_angle_deg = (float) (360 * WHEEL_RADIUS) * (tick_count_left - tick_count_right) / (WHEEL_SEPERATION * 4 * PULSE_PER_REV);
    angle_delta = cumulative_angle_deg - prev_cumulative_angle_deg;

    // angle error calculate
    float angle_error = set_angle - cumulative_angle_deg;

    // calculate speed for each motor
    float constant_speed_term = WHEEL_SEPERATION * angle_error * PI / (CONTROL_UPDATE_RATE * 360);

    left_set_speed  =  constant_speed_term + set_velocity;
    right_set_speed = -constant_speed_term + set_velocity;
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
