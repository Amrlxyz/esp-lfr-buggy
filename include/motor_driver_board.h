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

/**
 * @brief Contains all the functionality of the ESP motor driver board.
 * 
 * Functionality:
 * - enable/disable the board using the "enable" pin
 * - read voltage and current used by the whole driver board
 * 
 */
class MotorDriverBoard
{
protected:
    
    DigitalOut board_enable;            ///< pin responsible for enabling the board
    bool enable_state;                  ///< state of the enable pin

    int VoltageReading, CurrentReading; ///< gets the value of voltage and current as integers per unit value referred to in the ds2781.cpp in more detail
    float Voltage, Current;             ///< actual value of the voltage and current

public:

    MotorDriverBoard(PinName enable_pin, PinName monitor_pin);

    /**
     * @brief updates the measurements of voltage and current values
     * 
     */
    void update_measurements(void);     

    /**
     * @brief enable pin will be set to the boolean value in the paremeter
     * 
     * @param expression 
     */
    void set_enable(bool expression);   

    /**
     * @brief causes the enable pin to be set to HIGH
     * 
     */
    void enable(void);                  
    
    /**
     * @brief causes the enable pin to be set to LOW
     * 
     */
    void disable(void);                 

    /**
     * @brief returns the actual voltage value
     * 
     * @return float 
     */
    float get_voltage(void);            

    /**
     * @brief returns the actual curent value
     * 
     * @return float 
     */
    float get_current(void);            

    /**
     * @brief returns the state of the enable pin
     * 
     * @return true 
     * @return false 
     */
    bool get_enable_state(void);
};
