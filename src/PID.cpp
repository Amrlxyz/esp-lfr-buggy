#include "mbed.h"
#include "PID.h"
#include "constants.h"


PID::PID(
        float kp_,
        float ki_,
        float kd_,
        float tau_,
        float lim_min_output_, 
        float lim_max_output_, 
        float lim_min_int_, 
        float lim_max_int_)
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
        sample_time = CONTROL_UPDATE_PERIOD;
    }


void PID::update(float set_point, float measurement) 
{
    /* Error */
    float error = set_point - measurement;


    /* --- PROPOTIONAL TERM ---  */
    float proportional = kp * error;

    
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
    output = proportional + ki * integrator + kd * differentiator;

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



// void PID::reset(void) 
// {
//     Kp = 0;
//     Ki = 0;
//     Kd = 0;
//     integrator = 0;
//     prevError = 0;
//     differentiator = 0;
//     prevposition = 0;
//     output = 0;
//     limMin = 0; 
//     limMax = 0; 
//     limMinInt= 0; 
//     limMaxInt = 0; 
//     setposition = 0;
// }