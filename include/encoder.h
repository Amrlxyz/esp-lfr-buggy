#pragma once

#include "mbed.h"
#include "constants.h"
#include "QEI.h"


class Encoder
{
protected:

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

    // Constructor, specify encoder pins
    Encoder(PinName CH_A, PinName CH_B);
    
    // Calculate and update all the speed variables
    // Preferably run as an ISR
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