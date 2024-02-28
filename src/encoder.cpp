#include "mbed.h"
#include "constants.h"
#include "QEI.h"
#include "encoder.h"


Encoder::Encoder(PinName CH_A, PinName CH_B): qei(CH_A, CH_B, NC, PULSE_PER_REV, QEI::X4_ENCODING) 
{
    prev_tick_count = 0;
    speed = 0;
    prev_speed = 0;
    filtered_speed = 0;
    prev_filtered_speed = 0;
}

void Encoder::update(void)
{
    // update pulse diff
    curr_tick_count = qei.getPulses();
    tick_diff = curr_tick_count - prev_tick_count;
    prev_tick_count = curr_tick_count;

    // update rotational freq
    rotational_freq = ((float) tick_diff / (4 * PULSE_PER_REV)) * CONTROL_UPDATE_RATE;

    // update rpm
    rpm = rotational_freq * 60;

    // calculate raw speed
    speed = 2 * PI * WHEEL_RADIUS * rotational_freq;

    // low pass filter for speed
    float b0 = 0.0591174;
    float b1 = 0.0591174;
    float a0 = 0.88176521;

    filtered_speed = prev_filtered_speed * a0 + speed * b0 + prev_speed * b1;

    prev_filtered_speed = filtered_speed;
    prev_speed = speed;
}

void Encoder::reset(void)
{
    qei.reset();
    curr_tick_count = 0;
}

int Encoder::get_tick_count(void)
{
    return curr_tick_count;
}

float Encoder::get_rotational_freq(void)
{
    return rotational_freq;
}

float Encoder::get_rpm(void)
{
    return rpm;
}

float Encoder::get_speed(void)
{
    return speed;
}

float Encoder::get_filtered_speed(void)
{
    return filtered_speed;
}
