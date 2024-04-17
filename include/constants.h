/**
 * @file constants.h
 * @brief contains all the constant variables for easy access from one file
 * 
 * All constants are in metres, seconds, hertz unless otherwise stated
 *  
 */


#pragma once

/* PID CONSTANTS */

// General Motor PID constants
#define PID_M_TAU           1
#define PID_M_MIN_OUT       -PID_M_MAX_OUT
#define PID_M_MAX_OUT       0.9
#define PID_M_MIN_INT       -PID_M_MAX_INT
#define PID_M_MAX_INT       0.9

// Left Motor PID Constants
#define PID_M_L_KP          0.5
#define PID_M_L_KI          10
#define PID_M_L_KD          0

// Right Motor PID Constants
#define PID_M_R_KP          PID_M_L_KP
#define PID_M_R_KI          PID_M_L_KI
#define PID_M_R_KD          PID_M_L_KD 

// Angle PID Constants
#define PID_A_TAU           1
#define PID_A_MIN_OUT       -PID_M_MAX_OUT
#define PID_A_MAX_OUT       0.6
#define PID_A_MIN_INT       -PID_M_MAX_INT
#define PID_A_MAX_INT       0.3
#define PID_A_KP            0.1
#define PID_A_KI            0
#define PID_A_KD            0


/* OTHER CONSTANTS */

// Sensor Array Constants
#define SENS_SAMPLE_COUNT       5   // 5 - 311us, 3 - 195us   
#define SENS_ANGLE_COEFF        -1
#define SENS_DETECT_THRESH      0.25

// Line Follow Constants
#define LINE_FOLLOW_VELOCITY        0.5
#define LINE_FOLLOW_STOP_DISTANCE   0.30

// Square Task Constants
#define SQUARE_VELOCITY_SET                 0.4
#define SQUARE_TURNING_RIGHT_ANGLE          92
#define SQUARE_TURNING_LEFT_ANGLE           101.5
#define SQUARE_DISTANCE                     1.01

// Control Timing Constants
#define CONTROL_UPDATE_RATE         100                                     // Hz
#define CONTROL_UPDATE_PERIOD       (1.0f / CONTROL_UPDATE_RATE)            // Seconds
#define CONTROL_UPDATE_PERIOD_US    (int)(1'000'000 / CONTROL_UPDATE_RATE)  // Micro Seconds

// Serial Update Timing Constants
#define SERIAL_UPDATE_PERIOD        1     /// Seconds

// Encoder Constants
#define WHEEL_SEPERATION    0.188       
#define WHEEL_RADIUS        0.0415      
#define PULSE_PER_REV       256         
#define LP_SPEED_B0         0.1802684       
#define LP_SPEED_B1         0.1802684       
#define LP_SPEED_A0         0.63946321      

// 2 Hz Pole Freq:
// Filter coefficients b_i: [0.0591174 0.0591174]
// Filter coefficients a_i: [0.88176521]
// 5 Hz Pole Freq:
// Filter coefficients b_i: [0.13575525 0.13575525]
// Filter coefficients a_i: [0.7284895]
// 7 Hz Pole Freq:
// Filter coefficients b_i: [0.1802684 0.1802684]
// Filter coefficients a_i: [0.63946321]

// Motor Constants
#define MOTOR_PWM_FREQ      20000

// Bluetooth HM10 module default config constants
#define BT_BAUD_RATE        9600

// Maths constant
#define PI                  3.14159265