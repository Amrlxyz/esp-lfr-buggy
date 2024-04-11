#include "mbed.h"
#include "PID.h"


PID::PID(
        float kp_,
        float ki_,
        float kd_,
        float tau_,
        float lim_min_output_, 
        float lim_max_output_, 
        float lim_min_int_, 
        float lim_max_int_,
        float update_period)
    {
        // Controller gains
        kp = kp_; 
        ki = ki_; 
        kd = kd_; 

        // Derivative low-pass filter time constant
        tau = tau_;

        // Output limits
        lim_min_output = lim_min_output_; 
        lim_max_output = lim_max_output_; 

        // Integrator Limits
        lim_min_int = lim_min_int_; 
        lim_max_int = lim_max_int_; 

        // Controller memory
        integrator = 0; 
        prev_error = 0;           
        differentiator = 0;
        prev_measurement = 0;     

        // controller output
        output = 0;

        // sample_time
        sample_time = update_period;
        time_index = 0;

        // set pointer to terms
        output_arr[0] = &time_index;
        output_arr[1] = &set_point;
        output_arr[2] = &measurement;
        output_arr[3] = &error;
        output_arr[4] = &proportional;
        output_arr[5] = &integrator;
        output_arr[6] = &differentiator;
        output_arr[7] = &output;
    }


void PID::update(float set_point_, float measurement_) 
{
    /* Error */
    error = set_point_ - measurement_;
    measurement = measurement_;
    set_point = set_point_;


    /* --- PROPOTIONAL TERM ---  */
    proportional = kp * error;

    
    /* --- INTEGRAL TERM ---  */
    integrator = integrator + 0.5f * ki * sample_time * (error + prev_error);  

    //  Anti-wind-up via integrator clamping 
    if (integrator > lim_max_int) 
    {
        integrator = lim_max_int;
    } 
    else if (integrator < lim_min_int) 
    {
        integrator = lim_min_int;
    }


    /* --- DERIVATIVE TERM --- */
    differentiator = -(2.0f * kd * (measurement - prev_measurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
                    + (2.0f * tau - sample_time) * differentiator)
                    / (2.0f * tau + sample_time);


    //  Compute Output
    output = proportional + integrator + differentiator;
    time_index += sample_time;

    // Apply limits
    if (output > lim_max_output) 
    {
        output = lim_max_output;
    } 
    else if (output < lim_min_output) 
    {
        output = lim_min_output;
    }
    
    // store values for future update
    prev_error = error;
    prev_measurement = measurement;
}    


float PID::get_output(void)
{
    return output;
}


void PID::reset(void) 
{
    integrator = 0;
    prev_error = 0;
    differentiator = 0;
    prev_measurement = 0;
    
    time_index = 0;
    set_point = 0;
    measurement = 0;
    error = 0;
    proportional = 0;
    integrator = 0;
    differentiator = 0;
    output = 0;
}


void PID::set_constants(float kp_, float ki_, float kd_)
{
    kp = kp_;
    ki = ki_;
    kd = kd_;
}

float** PID::get_terms(void)
{
    return output_arr;
}