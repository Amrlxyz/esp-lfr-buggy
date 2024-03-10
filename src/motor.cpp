#include "mbed.h"

#include "constants.h"
#include "QEI.h"
#include "motor.h"


Motor::Motor(PinName pwm, PinName dir, PinName bip, PinName CH_A, PinName CH_B): PWM_pin(pwm), Direction(dir), Bipolar(bip), qei(CH_A, CH_B, NC, PULSE_PER_REV, QEI::X4_ENCODING)
{
    PWM_pin.period(1.0 / MOTOR_PWM_FREQ);
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
    rotational_freq = ((float) tick_diff / (4 * PULSE_PER_REV)) * CONTROL_UPDATE_RATE;

    // update rpm
    rpm = rotational_freq * 60;

    // calculate raw speed
    speed = 2 * PI * WHEEL_RADIUS * rotational_freq;

    // low pass filter for speed
    filtered_speed = (prev_filtered_speed * LP_SPEED_A0) + (speed * LP_SPEED_B0) + (prev_speed * LP_SPEED_B1);

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