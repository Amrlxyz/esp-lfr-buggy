#include "mbed.h"
#include "motor.h"


Motor::Motor(PinName pwm, PinName dir, PinName bip): PWM_pin(pwm), Direction(dir), Bipolar(bip)
{
    PWM_pin.period(1.0 / MOTOR_PWM_FREQ);
    set_bipolar_mode(false);
    set_duty_cycle(0);
};

void Motor::set_direction(int DirState)
{
    Direction.write(DirState);
};
    
void Motor::set_bipolar_mode(bool BipState)
{
    Bipolar.write(BipState); 
};

void Motor::set_duty_cycle(float DutyCycle)
{
    PWM_pin.write(DutyCycle); 
};