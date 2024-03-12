#pragma once

/* PID CONSTANTS */

// General Motor PID constants
#define PID_M_TAU           1
#define PID_M_MIN_OUT       -PID_M_MAX_OUT
#define PID_M_MAX_OUT       0.5
#define PID_M_MIN_INT       -PID_M_MAX_INT
#define PID_M_MAX_INT       0.4

// Left Motor PID Constants
#define PID_M_L_KP          1
#define PID_M_L_KI          2
#define PID_M_L_KD          0

// Right Motor PID Constants
#define PID_M_R_KP          PID_M_L_KP
#define PID_M_R_KI          PID_M_L_KI
#define PID_M_R_KD          PID_M_L_KD 

// Angle PID Constants
#define PID_A_TAU           1
#define PID_A_MIN_OUT       -PID_M_MAX_OUT
#define PID_A_MAX_OUT       0.3
#define PID_A_MIN_INT       -PID_M_MAX_INT
#define PID_A_MAX_INT       0.3
#define PID_A_KP            0.1
#define PID_A_KI            0
#define PID_A_KD            0


/* OTHER CONSTANTS */

// Sensor Array Constants
#define SENS_SAMPLE_COUNT       5
#define SENS_ANGLE_COEFF        -15

// Line Follow Constants
#define LINE_FOLLOW_VELOCITY    0.5

// Square Task Constants
#define SQUARE_VELOCITY_SET                 0.4
#define SQUARE_TURNING_RIGHT_ANGLE          92
#define SQUARE_TURNING_LEFT_ANGLE           101.5
#define SQUARE_DISTANCE                     1.01

// Control Timing Constants
#define CONTROL_UPDATE_RATE         10                                     // Hz
#define CONTROL_UPDATE_PERIOD       (1.0f / CONTROL_UPDATE_RATE)            // Seconds
#define CONTROL_UPDATE_PERIOD_US    (int)(1'000'000 / CONTROL_UPDATE_RATE)  // Micro Seconds

// Serial Update Timing Constants
#define SERIAL_UPDATE_PERIOD        .1     // Seconds

// Encoder Constants
#define WHEEL_SEPERATION    0.188       // metres
#define WHEEL_RADIUS        0.0415      // metres
#define PULSE_PER_REV       256         
#define LP_SPEED_B0         0.0591174
#define LP_SPEED_B1         0.0591174
#define LP_SPEED_A0         0.88176521

// Motor Constants
#define MOTOR_PWM_FREQ      20'000

// Bluetooth HM10 module default config constants
#define BT_BUFFER_SIZE      20
#define BT_BAUD_RATE        9600

// Maths constant
#define PI                  3.14159265