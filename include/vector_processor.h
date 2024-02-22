#pragma once

#include "mbed.h"
#include "constants.h"


class VectorProcessor
{
protected:
    
    volatile float prev_cumulative_angle_deg;
    volatile float cumulative_angle_deg;
    volatile float angle_delta;
    volatile float left_set_speed;
    volatile float right_set_speed;
    float set_velocity; 
    float set_angle;
    float distance_travelled;

public:

    VectorProcessor(void);
    
    void update(float tick_count_left, float tick_count_right);

    void reset_distance_travelled(void);

    float get_distance_travelled(void);
    
    float get_cumulative_angle_deg(void);

    float get_angle_delta(void);

    float get_left_set_speed(void);

    float get_right_set_speed(void);

    void set_set_velocity(float vel);

    void set_set_angle(float ang);
};