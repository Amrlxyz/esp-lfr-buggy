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
    //Digital output which controls the state of the TCRT5000's infrared LEDs
    DigitalOut led0_;
    DigitalOut led1_;
    DigitalOut led2_;
    DigitalOut led3_;
    DigitalOut led4_;
    DigitalOut led5_;
    //Analog input from the TCRT5000's phototransistor
    AnalogIn sens0_;
    AnalogIn sens1_;
    AnalogIn sens2_;
    AnalogIn sens3_;
    AnalogIn sens4_;
    AnalogIn sens5_;
    //calculated outputs from sensor array
    float output; 
    float angle_output;

    float sens_values[6]; //values of the sensors in an array
    
    const int sample_count_; //total sample counted for the moving average

    float read(AnalogIn sensor);


public:

    SensorArray(PinName sens0, PinName sens1, PinName sens2, PinName sens3, PinName sens4, PinName sens5,
                PinName led0, PinName led1, PinName led2, PinName led3, PinName led4, PinName led5, int sample_count);

    
    void reset(void); //resets values in sensor array and all outputs to 0

    
    void set_all_led_on(bool status); //turns on all sensor LEDs

    
    void update(void); //updates the values in the sensor array and the outputs

    
    float get_sens_output(int index); //returns value of the indexed element of the sensor array


    float* get_sens_output_array(void); //returns a pointer to the sensor array


    float get_array_output(void); //returns calculated sensor array output


    float get_angle_output(void); //returns calculated sensor array angle output
};