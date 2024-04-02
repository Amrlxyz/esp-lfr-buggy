/**
 * @file PID.h
 * @brief PID class library
 * 
 * 
 */

#pragma once

#include "mbed.h"

class PID
{
private:

    // Controller gains
    float kp; 
    float ki; 
    float kd; 

    // Derivative low-pass filter time constant
    /*  The derivative low-pass filter can be controlled by the constant 'tau', 
        which is the time constant of the filter (-3dB frequency in Hz, fc = 1 / (2*pi*tau)).
        A larger value of tau means the signal is filtered more heavily. 
        As tau approaches zero, the differentiator approaches a 'pure differentiator' with no filtering. */
    float tau;

    // Output limits
    float lim_min_output; 
    float lim_max_output; 

    // Integrator Limits
    float lim_min_int; 
    float lim_max_int; 

    // Controller memory
    float integrator; 
    float prev_error;           // Required by Integrator 
    float differentiator;
    float prev_measurement;     // Required by Differentiator

    // Terms
    float time_index;
    float set_point;
    float error;
    float measurement;
    float proportional;
    float output_arr[8];

    // controller output
    float output;

    // sample time (secs)
    float sample_time;

public:

    PID(float kp_,
        float ki_,
        float kd_,
        float tau_,
        float lim_min_output_, 
        float lim_max_output_, 
        float lim_min_int_, 
        float lim_max_int_);

    float* get_terms(void);

    void update(float set_point, float measurement);

    void set_constants(float kp_, float ki_, float kd_);

    void reset();

    float get_output(void);
};