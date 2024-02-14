#include "mbed.h"
#include "QEI.h"




class Motor{
    private:
        PwmOut PWM_pin;
        DigitalOut Direction, Bipolar;

    public:
    Motor(PinName pwm, PinName dir, PinName bip):PWM_pin(pwm), Direction(dir), Bipolar(bip){
        PWM_pin.period_ms(4);
    };

    void direction_mode(int DirState){
        Direction.write(DirState); //controls the direction
    };
    void set_bipolar_mode(int BipState){
        Bipolar.write(BipState); //controls whether its bipolar or unipolar PWM operation 
    };
    void set_duty_cycle(float DutyCycle){
        PWM_pin.write(DutyCycle); //sets the duty cycle
    };
};
