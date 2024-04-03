/**
 * @file sensor_array.h
 * @brief Sensor Array PCB interface class library  
 * 
 * 
 */

#pragma once

#include "mbed.h"


class SensorArray
{
protected:

    DigitalOut led[6];
    AnalogIn sens[6];

    float output;
    float sens_values[6];
    
    const int sample_count_;
    const float detect_thresh_;
    bool line_detected;

    float read(AnalogIn sensor); // WIP

public:

    SensorArray(PinName sens0, PinName sens1, PinName sens2, PinName sens3, PinName sens4, PinName sens5,
                PinName led0, PinName led1, PinName led2, PinName led3, PinName led4, PinName led5, int sample_count, float detect_thresh);

    
    void reset(void);

    
    void update(void);


    bool is_line_detected(void);


    void set_all_led_on(bool status);

    
    float get_sens_output(int index);


    float* get_sens_output_array(void);


    float get_array_output(void);
};