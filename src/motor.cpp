#include "mbed.h"
#include "motor.h"


Motor::Motor(PinName pwm, PinName dir, PinName bip): PWM_pin(pwm), Direction(dir), Bipolar(bip)
{
    PWM_pin.period(1.0 / MOTOR_PWM_FREQ);
    set_direction(1);
    set_bipolar_mode(false);
    set_duty_cycle(0);
};

void Motor::set_direction(int DirState)
{
    Direction.write(DirState);
    direction = DirState;
};
    
void Motor::set_bipolar_mode(bool BipState)
{
    Bipolar.write(BipState);
    bipolar = BipState; 
};

void Motor::set_duty_cycle(float DutyCycle)
{
    duty_cycle = 1 - DutyCycle;
    PWM_pin.write(duty_cycle); 
};

bool Motor::get_direction(void)
{
    return direction;
};

bool Motor::get_bipolar(void)
{
    return bipolar;
};

bool Motor::get_duty_cycle(void)
{
    return duty_cycle;
};