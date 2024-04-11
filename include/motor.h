/**
 * @file motor.h
 * @brief Motor and Encoder class library
 * 
 * 
 */

#pragma once

#include "mbed.h"
#include "QEI.h"

/**
 * @brief Represents a motor with integrated quadrature encoder for the buggy.
 * 
 * This class provides functionality to control the speed and direction of the motor,
 * read encoder tick counts, and calculate various parameters such as rotational frequency,
 * speed, and revolutions per minute (RPM). It also includes a low-pass filter for noise reduction
 * in speed measurements.
 */
class Motor
{
private:

    PwmOut PWM_pin;                 // creates a pulse-width-modulated(PWM) pin
    DigitalOut Direction, Bipolar;  // pins responsible for controlling direction and whether the H-bridge is bipolar or unipolar 
    float duty_cycle;               // duty cycle of the PWM
    bool direction;                 // boolean state of the direction pin
    bool bipolar;                   // boolean state of the bipolar pin where: HIGH means its bipolar and LOW means its unipolar

    QEI qei;                            // quadrature encoder object from an imported library
    volatile int curr_tick_count;       // the latest cumulative tick count recorded
    volatile int prev_tick_count;       // the cumulative tick count recorded before the latest cumulative tick count
    volatile float rotational_freq;     // rotational frequency of the wheel
    volatile float speed;               // latest tangential speed of the wheel before filtering out the noise
    volatile float filtered_speed;      // latest tangential speed of the wheel after filtering out the noise
    volatile float prev_speed;          // previous tangential speed of the wheel after filtering out the noise
    volatile float prev_filtered_speed; // previous tangential speed of the wheel before filtering out the noise
    volatile float rpm;                 // latest rounds per minute of the wheel

    const int pwm_freq;         // frequency at which the PWM is operating at
    const int update_rate;      // rate of which the values are updated
    const int pulse_per_rev;    // the tick counts counted by the encoder per revolution
    const float wheel_radius;   // radius of the buggy wheel

    // Low pass filter constants
    const float LP_a0; 
    const float LP_b0;
    const float LP_b1;

    // value of pi used
    const float pi = 3.14159265;

public:

    /**
     * @brief Construct a new Motor object.
     * 
     * @param pwm The PWM pin.
     * @param dir The direction pin.
     * @param bip The bipolar/unipolar mode pin.
     * @param CH_A The encoder channel A pin.
     * @param CH_B The encoder channel B pin.
     * @param pulsePerRev The number of pulses per revolution of the motor.
     * @param pwmFreq The PWM frequency.
     * @param updateRate The update rate.
     * @param LowPass_a0 Coefficient a0 for the low-pass filter.
     * @param LowPass_b0 Coefficient b0 for the low-pass filter.
     * @param LowPass_b1 Coefficient b1 for the low-pass filter.
     * @param wheelRadius The radius of the wheel in meters.
     */
    Motor(PinName pwm, PinName dir, PinName bip, PinName CH_A, PinName CH_B,
          int pulsePerRev, int pwmFreq, int updateRate, float LowPass_a0, float LowPass_b0, float LowPass_b1, float wheelRadius);

    /**
     * @brief Set the direction of the motor.
     * 
     * @param DirState The direction state (true for forward, false for reverse).
     */
    void set_direction(bool DirState);

    /**
     * @brief Set the bipolar mode of the motor.
     * 
     * @param BipState The bipolar mode state (true for bipolar mode, false for unipolar mode).
     */
    void set_bipolar_mode(bool BipState);

    /**
     * @brief Set the duty cycle of the motor.
     * 
     * @param DutyCycle The duty cycle value (0 to 1).
     */
    void set_duty_cycle(float DutyCycle);

    /**
     * @brief Get the direction of the motor.
     * 
     * @return The direction state (true for forward, false for reverse).
     */
    bool get_direction();

    /**
     * @brief Get the bipolar mode of the motor.
     * 
     * @return The bipolar mode state (true for bipolar mode, false for unipolar mode).
     */
    bool get_bipolar_mode();

    /**
     * @brief Get the duty cycle of the motor.
     * 
     * @return The duty cycle value (0 to 1).
     */
    float get_duty_cycle();

    // Encoder Stuffs:

    /**
     * @brief Calculate and update all the speed variables.
     * 
     * Preferably run in an Interrupt Service Routine (ISR).
     */
    void update(void);

    /**
     * @brief Reset the encoder tick count.
     */
    void reset(void);

    /**
     * @brief Get the cumulative tick count.
     * 
     * @return The cumulative tick count.
     */
    int get_tick_count(void);

    /**
     * @brief Get the rotational frequency of the motor in revolutions per second (rev/s).
     * 
     * @return The rotational frequency.
     */
    float get_rotational_freq(void);

    /**
     * @brief Get the revolutions per minute (RPM) of the motor.
     * 
     * @return The RPM value.
     */
    float get_rpm(void);

    /**
     * @brief Get the tangential speed of the wheel.
     * 
     * @return The tangential speed.
     */
    float get_speed(void);

    /**
     * @brief Get the low-pass filtered speed of the wheel.
     * 
     * @return The low-pass filtered speed.
     */
    float get_filtered_speed(void);

};
