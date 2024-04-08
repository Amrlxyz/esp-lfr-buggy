/**
 * @file motor.h
 * @brief Motor and Encoder class library
 * 
 * 
 */

#pragma once

#include "mbed.h"
#include "QEI.h"

class Motor
{
private:

    PwmOut PWM_pin; //creates a pulse-width-modulated(PWM) pin
    DigitalOut Direction, Bipolar; //pins responsible for controlling direction and whether the H-bridge is bipolar or unipolar 
    float duty_cycle; //duty cycle of the PWM
    bool direction; //boolean state of the direction pin
    bool bipolar; //boolean state of the bipolar pin where: HIGH means its bipolar and LOW means its unipolar

    QEI qei; //quadrature encoder object from an imported library
    volatile int curr_tick_count; //the latest cumulative tick count recorded
    volatile int prev_tick_count; //the cumulative tick count recorded before the latest cumulative tick count
    volatile float rotational_freq; //rotational frequency of the wheel
    volatile float speed; //latest tangential speed of the wheel before filtering out the noise
    volatile float filtered_speed; //latest tangential speed of the wheel after filtering out the noise
    volatile float prev_speed;  //previous tangential speed of the wheel after filtering out the noise
    volatile float prev_filtered_speed; //previous tangential speed of the wheel before filtering out the noise
    volatile float rpm; //latest rounds per minute of the wheel

    const int pwm_freq; //frequency at which the PWM is operating at
    const int update_rate; //rate of which the values are updated
    const int pulse_per_rev; //the tick counts counted by the encoder per revolution
    const float wheel_radius; //radius of the buggy wheel
    //Low pass filter constants
    const float LP_a0; 
    const float LP_b0;
    const float LP_b1;
    
    const float pi = 3.14159265; //value of pi used

public:

    Motor(PinName pwm, PinName dir, PinName bip, PinName CH_A, PinName CH_B, 
            int pulsePerRev, int pwmFreq, int updateRate, float LowPass_a0, float LowPass_b0, float LowPass_b1, float wheelRadius);

    // controls the motor direction
    void set_direction(bool DirState);
    
    // controls whether its bipolar or unipolar PWM operation 
    void set_bipolar_mode(bool BipState);

    // sets the duty cycle
    void set_duty_cycle(float DutyCycle);

    bool get_direction();

    bool get_bipolar_mode();

    float get_duty_cycle();


    // Encoder Stuffs:
    
    // Calculate and update all the speed variables
    // Preferably run in an ISR
    void update(void);

    // Reset the encoder tick count
    void reset(void);

    // Returns the cumulative tick counts
    int get_tick_count(void);

    // Returns rotational freq (rev per sec)
    float get_rotational_freq(void);

    // Returns wheel RPM
    float get_rpm(void);

    // Returns the wheel tangential speed
    float get_speed(void);

    // Returns the lowpass-filtered speed 
    float get_filtered_speed(void);

};
