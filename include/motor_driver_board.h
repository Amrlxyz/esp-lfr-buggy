/**
 * @file motor_driver_board.h
 * @brief ESP Motor Driver Board Interface library  
 * 
 * 
 */

#pragma once

#include "mbed.h"

#include "ds2781.h"
#include "OneWire_Methods.h"


class MotorDriverBoard
{
protected:
    
    DigitalOut board_enable; //pin responsible for enabling the board
    bool enable_state; //state of the enable pin

    int VoltageReading, CurrentReading; //gets the value of voltage and current as integers per unit value referred to in the ds2781.cpp in more detail
    float Voltage, Current; //actual value of the voltage and current

public:

    MotorDriverBoard(PinName enable_pin, PinName monitor_pin);

    void update_measurements(void); //updates the measurements of voltage and current values

    void set_enable(bool expression); //enable pin will be set to the boolean value in the paremeter

    void enable(void); //causes the enable pin to be set to HIGH
    
    void disable(void); //causes the enable pin to be set to LOW

    float get_voltage(void); //returns the actual voltage value

    float get_current(void); //returns the actual cuurent value

    bool get_enable_state(void); //returns the state of the enable pin
};
