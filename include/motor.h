#pragma once

#include "mbed.h"
#include "QEI.h"

class Motor
{
private:

    PwmOut PWM_pin;
    DigitalOut Direction, Bipolar;
    float duty_cycle;
    bool direction;
    bool bipolar;

    QEI qei;
    volatile int curr_tick_count;
    volatile int prev_tick_count;
    volatile float rotational_freq;
    volatile float speed;
    volatile float filtered_speed;
    volatile float prev_speed;
    volatile float prev_filtered_speed;
    volatile float rpm;

public:

    Motor(PinName pwm, PinName dir, PinName bip, PinName CH_A, PinName CH_B);

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
