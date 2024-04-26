/**
 * @file PID.h
 * @brief PID class library
 * 
 * 
 */

#pragma once

#include "mbed.h"

/**
 * @brief PID Control Class
 * 
 * Requires manual update of output using update() function
 * 
 * Assumes a constant period between updates, so a ticker that calls update() is recommended
 * 
 */
class PID
{
private:

    // Controller gains
    float kp; 
    float ki; 
    float kd; 

    /**
     * @brief Derivative low-pass filter time constant
     * 
     * The derivative low-pass filter can be controlled by the constant 'tau', 
     * which is the time constant of the filter (-3dB frequency in Hz, fc = 1 / (2*pi*tau)).
     * A larger value of tau means the signal is filtered more heavily. 
     * As tau approaches zero, the differentiator approaches a 'pure differentiator' with no filtering.
     * 
     */
    float tau;

    // output limit
    float lim_min_output;       
    float lim_max_output;       

    // Integrator term limit
    float lim_min_int;          
    float lim_max_int;          

    // Controller memory
    float integrator;           // Integrator term
    float prev_error;           // Required by Integrator 
    float differentiator;       // Integrator term
    float prev_measurement;     // Required by Differentiator

    // PID Terms and Variables
    float time_index;
    float set_point;
    float error;
    float measurement;
    float proportional;
    float *output_arr[8];
    float constants_arr[4];

    // Controller output
    float output;

    // Sample time (secs)
    float sample_time;

public:

    /**
     * @brief Construct a new PID object
     * 
     * @param kp_ Propotional Term Gain Constant
     * @param ki_ Integral Term Gain Constant
     * @param kd_ Differentiator Term Gain Constant
     * @param tau_ Time Constant Tau
     * @param lim_min_output_ min output limit 
     * @param lim_max_output_ max output limit
     * @param lim_min_int_ min integral term limit
     * @param lim_max_int_ max integral term limit
     * @param update_period update period (secs)
     */
    PID(float kp_,
        float ki_,
        float kd_,
        float tau_,
        float lim_min_output_, 
        float lim_max_output_, 
        float lim_min_int_, 
        float lim_max_int_,
        float update_period);

    /**
     * @brief Get the PID terms.
     * 
     * @return A pointer to an array of floats containing 8 PID terms in the order:
     * 0. Time index,
     * 1. set point, 
     * 2. sensor measured value, 
     * 3. error, 
     * 4. propotional term
     * 5. integral term
     * 6. differentiator term
     * 7. output of the PID
     * 
     */
    float** get_terms(void);

    /**
     * @brief Updates the output of PID controller based on the real time measurement
     * 
     * @param set_point The desired set point.
     * @param measurement The current measurement.
     */
    void update(float set_point, float measurement);

    /**
     * @brief Takes in the 3 parameters to set the PID coefficients values.
     * 
     * @param kp_ Propotional Term Gain Constant.
     * @param ki_ Integral Term Gain Constant.
     * @param kd_ Differentiator Term Gain Constant.
     */
    void set_constants(float kp_, float ki_, float kd_);

    void set_tau(float tau_);

    /**
     * @brief Reset the PID controller.
     * 
     * This function resets the integral term and clears any internal state of the PID controller.
     */
    void reset();

    /**
     * @brief Get the output of the PID controller.
     * 
     * @return The output of the PID controller.
     */
    float get_output(void);

    float* get_constants(void);
};