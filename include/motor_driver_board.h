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
 * Uses this Library by Sam Walsh to interface with the current/voltage IC.
 * https://os.mbed.com/users/EmbeddedSam/code/Nucleo_F401RE_DS271_Battery_Monitor/
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

    /**
     * @brief Construct a new MotorDriverBoard object
     * 
     * @param enable_pin motor driver board enable (EN) pin
     * @param monitor_pin motor driver board monitor (OneWire) pin
     */
    MotorDriverBoard(PinName enable_pin, PinName monitor_pin);

    /**
     * @brief updates the measurements of voltage and current values
     * 
     */
    void update_measurements(void);     

    /**
     * @brief enable pin will be set to the boolean value in the paremeter
     * 
     * @param expression boolean value or operation to be applied
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
     * @return float voltage
     */
    float get_voltage(void);            

    /**
     * @brief returns the actual curent value
     * 
     * @return float current
     */
    float get_current(void);            

    /**
     * @brief returns the state of the enable pin
     * 
     * @return true if pin is enabled
     * @return false if pin is disabled
     */
    bool get_enable_state(void);
};
