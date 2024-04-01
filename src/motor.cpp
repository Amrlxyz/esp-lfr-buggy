#include "mbed.h"

#include "QEI.h"
#include "motor.h"


Motor::Motor(PinName pwm, PinName dir, PinName bip, PinName CH_A, PinName CH_B, 
             int pulsePerRev, int pwmFreq, int updateRate, float LowPass_a0, float LowPass_b0, float LowPass_b1, float wheelRadius): 
                PWM_pin(pwm), 
                Direction(dir), 
                Bipolar(bip), 
                qei(CH_A, CH_B, NC, pulsePerRev, QEI::X4_ENCODING),
                pulse_per_rev(pulsePerRev),
                pwm_freq(pwmFreq),
                update_rate(updateRate), 
                LP_a0(LowPass_a0),
                LP_b0(LowPass_b0),
                LP_b1(LowPass_b1),
                wheel_radius(wheelRadius)
{
    PWM_pin.period(1.0 / pwm_freq);
    set_direction(1);
    set_bipolar_mode(false);
    set_duty_cycle(0);

    prev_tick_count = 0;
    speed = 0;
    prev_speed = 0;
    filtered_speed = 0;
    prev_filtered_speed = 0;
};

void Motor::update(void)
{
    // update pulse diff
    curr_tick_count = qei.getPulses();
    int tick_diff = curr_tick_count - prev_tick_count;
    prev_tick_count = curr_tick_count;

    // update rotational freq
    rotational_freq = ((float) tick_diff / (4 * pulse_per_rev)) * update_rate;

    // update rpm
    rpm = rotational_freq * 60;

    // calculate raw speed
    speed = 2 * pi * wheel_radius * rotational_freq;

    // low pass filter for speed
    filtered_speed = (prev_filtered_speed * LP_a0) + (speed * LP_b0) + (prev_speed * LP_b1);

    prev_filtered_speed = filtered_speed;
    prev_speed = speed;
}

void Motor::reset(void)
{
    qei.reset();

    curr_tick_count = 0;
    prev_tick_count = 0;
    rotational_freq = 0;
    speed = 0;
    filtered_speed = 0;
    prev_speed = 0;
    prev_filtered_speed = 0;
    rpm = 0;
}

void Motor::set_duty_cycle(float DutyCycle)
{
    duty_cycle = DutyCycle;
    if (duty_cycle < 0.0)
    {
        duty_cycle = -duty_cycle;
        set_direction(0);
    }
    else
    {
        set_direction(1);
    }
    PWM_pin.write(1 - duty_cycle);
};

void Motor::set_direction(bool DirState)
{
    Direction.write(DirState);
    direction = DirState;
};
    
void Motor::set_bipolar_mode(bool BipState)
{
    Bipolar.write(BipState);
    bipolar = BipState; 
};

bool Motor::get_direction(void)
{
    return direction;
};

bool Motor::get_bipolar_mode(void)
{
    return bipolar;
};

float Motor::get_duty_cycle(void)
{
    return duty_cycle;
};

int Motor::get_tick_count(void)
{
    return curr_tick_count;
}

float Motor::get_rotational_freq(void)
{
    return rotational_freq;
}

float Motor::get_rpm(void)
{
    return rpm;
}

float Motor::get_speed(void)
{
    return speed;
}

float Motor::get_filtered_speed(void)
{
    return filtered_speed;
}