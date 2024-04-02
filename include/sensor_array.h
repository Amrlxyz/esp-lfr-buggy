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

    DigitalOut led0_;
    DigitalOut led1_;
    DigitalOut led2_;
    DigitalOut led3_;
    DigitalOut led4_;
    DigitalOut led5_;

    AnalogIn sens0_;
    AnalogIn sens1_;
    AnalogIn sens2_;
    AnalogIn sens3_;
    AnalogIn sens4_;
    AnalogIn sens5_;

    float output;
    float angle_output;
    float sens_values[6];
    
    const int sample_count_;

    float read(AnalogIn sensor);


public:

    SensorArray(PinName sens0, PinName sens1, PinName sens2, PinName sens3, PinName sens4, PinName sens5,
                PinName led0, PinName led1, PinName led2, PinName led3, PinName led4, PinName led5, int sample_count);

    
    void reset(void);

    
    void set_all_led_on(bool status);

    
    void update(void);

    
    float get_sens_output(int index);


    float* get_sens_output_array(void);


    float get_array_output(void);


    float get_angle_output(void);
};