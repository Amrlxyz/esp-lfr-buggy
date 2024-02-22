#include "mbed.h"
#include "constants.h"
#include "QEI.h"
#include "encoder.h"


Encoder::Encoder(PinName CH_A, PinName CH_B): qei(CH_A, CH_B, NC, PULSE_PER_REV, QEI::X4_ENCODING) 
{
    prev_tick_count = 0;
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

    // update speed
    speed = 2 * PI * WHEEL_RADIUS * rotational_freq;
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
