#pragma once

#include "mbed.h"
#include "constants.h"


class Motor
{
private:

    PwmOut PWM_pin;
    DigitalOut Direction, Bipolar;
    float duty_cycle;
    bool direction;
    bool bipolar;

public:

    Motor(PinName pwm, PinName dir, PinName bip);

    // controls the motor direction
    void set_direction(bool DirState);
    
    // controls whether its bipolar or unipolar PWM operation 
    void set_bipolar_mode(bool BipState);

    // sets the duty cycle
    void set_duty_cycle(float DutyCycle);

    bool get_direction();

    bool get_bipolar();

    float get_duty_cycle();

};
