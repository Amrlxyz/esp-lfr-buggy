/**
 * @file sensor_array.h
 * @brief Sensor Array PCB interface class library  
 * 
 */

#pragma once

#include "mbed.h"


/**
 * @brief Represents an array of sensors with corresponding LEDs for detection.
 * 
 * This class provides functionality to read sensor values, detect lines, and control LEDs.
 */
class SensorArray
{
private:

    DigitalOut led[6];  // Array of DigitalOut objects to control the LEDs.
    AnalogIn sens[6];   // Array of AnalogIn objects to read the sensors.

    float output;           // The output value of the sensor array. 
    float sens_values[6];   // Array to store sensor values. 
    
    const int sample_count_;    // The number of samples to take for averaging sensor readings.
    const float detect_thresh_; // The detection threshold for line detection. 
    const float angle_coeff;    // The gain at which the sensor output is multiplied to represent the angle.
    bool line_detected;         // Flag indicating whether a line is detected. 

    const int coef[6] = {15, 9, 3, -3, -9, -15};

    /**
     * @brief Reads the value from the specified AnalogIn sensor.
     * 
     * @param sensor The AnalogIn sensor to read from.
     * @return The sensor value.
     */
    float read(AnalogIn sensor);

public:

    /**
     * @brief Constructs a new SensorArray object.
     * 
     * @param sens0-sens5 Pin names for the sensors.
     * @param led0-led5 Pin names for the LEDs.
     * @param sample_count The number of samples to take for averaging sensor readings.
     * @param detect_thresh The detection threshold for line detection.
     * @param angle_coefficient The gain at which the sensor output is multiplied to represent the angle.
     */
    SensorArray(PinName sens0, PinName sens1, PinName sens2, PinName sens3, PinName sens4, PinName sens5,
                PinName led0, PinName led1, PinName led2, PinName led3, PinName led4, PinName led5, int sample_count, float detect_thresh, float angle_coefficient);

    /**
     * @brief Resets the sensor array.
     * 
     * This function resets the internal state of the sensor array.
     */
    void reset(void);

    /**
     * @brief Updates the sensor array.
     * 
     * This function updates the sensor readings.
     */
    void update(void);

    /**
     * @brief Checks if a line is detected (in the last update).
     * 
     * @return True if a line is detected, false otherwise.
     */
    bool is_line_detected(void);

    /**
     * @brief Sets the status of all LEDs.
     * 
     * @param status The status to set (true for on, false for off).
     */
    void set_all_led_on(bool status);

    /**
     * @brief Gets the output value of a sensor at the specified index.
     * 
     * @param index The index of the sensor.
     * @return The output value of the sensor.
     */
    float get_sens_output(int index);

    /**
     * @brief Gets an array of sensor output values.
     * 
     * @return A pointer to an array of sensor output values.
     */
    float* get_sens_output_array(void);

    /**
     * @brief Gets the output value of the sensor array.
     * 
     * @return The output value of the sensor array.
     */
    float get_array_output(void);
};