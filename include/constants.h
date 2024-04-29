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
#define PID_M_MAX_OUT       1
#define PID_M_MIN_INT       -PID_M_MAX_INT
#define PID_M_MAX_INT       1

// Left Motor (M_L) PID Constants
#define PID_M_L_KP          0.5
#define PID_M_L_KI          7.5
#define PID_M_L_KD          0

// Right Motor (M_R) PID Constants
#define PID_M_R_KP          PID_M_L_KP
#define PID_M_R_KI          PID_M_L_KI
#define PID_M_R_KD          PID_M_L_KD 

// Sensor (S) PID Constants - USED FOR LINE FOLLOW ///////////////////////////
#define PID_S_MIN_OUT       -PID_S_MAX_OUT
#define PID_S_MAX_OUT       1.5
#define PID_S_MIN_INT       -PID_S_MAX_INT
#define PID_S_MAX_INT       0
#define PID_S_KI            0

#define PID_S_TAU           0.001
#define PID_S_KP            0.30
#define PID_S_KD            0.08

// 0.4 0.15 - Oscillates high freq when 0 - might need LP

// Angle (A) PID Constants - USED FOR TURNING
#define PID_A_TAU           0.01
#define PID_A_MIN_OUT       -PID_A_MAX_OUT
#define PID_A_MAX_OUT       0.6
#define PID_A_MIN_INT       -PID_A_MAX_INT
#define PID_A_MAX_INT       0
#define PID_A_KP            0.2
#define PID_A_KI            0.1
#define PID_A_KD            0


/* OTHER CONSTANTS */

// Sensor Array Constants
#define SENS_SAMPLE_COUNT       1   // 5 - 311us, 3 - 195us   
#define SENS_ANGLE_COEFF        1
#define SENS_DETECT_THRESH      0.10

// Line Follow Constants
#define LINE_FOLLOW_VELOCITY        1.5
#define LINE_FOLLOW_STOP_DISTANCE   0.3
#define UTURN_ANGLE                 210

#define SLOW_ACCEL_TIME             0.4
#define SLOW_ACCEL_DIVIDER          2

#define SLOW_TURNING_GAIN           0                       // Higher Means FASTER
#define SLOW_TURNING_THRESH         2

#define MANUAL_ACCEL_SPEED         0.5
#define MANUAL_ACCEL_DISTANCE      0.5
#define MANUAL_ACCEL_ANGLE         60


// Square Task Constants
#define SQUARE_VELOCITY_SET                 0.4
#define SQUARE_TURNING_RIGHT_ANGLE          92
#define SQUARE_TURNING_LEFT_ANGLE           101.5
#define SQUARE_DISTANCE                     1.01

// Control Timing Constants
#define CONTROL_UPDATE_RATE         2500                                    // Hz
#define CONTROL_UPDATE_PERIOD       (1.0f / CONTROL_UPDATE_RATE)            // Seconds
#define CONTROL_UPDATE_PERIOD_US    (int)(1'000'000 / CONTROL_UPDATE_RATE)  // Micro Seconds

// Control Timing Constants
#define SENSOR_UPDATE_RATE         5000                                    // Hz
#define SENSOR_UPDATE_PERIOD       (1.0f / CONTROL_UPDATE_RATE)            // Seconds
#define SENSOR_UPDATE_PERIOD_US    (int)(1'000'000 / CONTROL_UPDATE_RATE)  // Micro Seconds

// Serial Update Timing Constants
#define SERIAL_UPDATE_PERIOD        0.02     /// Seconds

#define LOG_SIZE                    8'500

// Encoder Constants
#define WHEEL_SEPERATION    0.188       
#define WHEEL_RADIUS        0.0415      
#define PULSE_PER_REV       256         
#define LP_SPEED_B0         0.13575525       
#define LP_SPEED_B1         0.13575525       
#define LP_SPEED_A0         0.7284895      

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