#pragma once

// general motor PID constants
#define PID_M_TAU           (2 * CONTROL_UPDATE_PERIOD)
#define PID_M_MIN_OUT       0.0
#define PID_M_MAX_OUT       0.7
#define PID_M_MIN_INT       -PID_M_MAX_INT
#define PID_M_MAX_INT       0.4

// Left Motor PID Constants
#define PID_M_L_KP          0.01          
#define PID_M_L_KI          0.0
#define PID_M_L_KD          0.0

// Right Motor PID Constants
#define PID_M_R_KP          0.01
#define PID_M_R_KI          0.0
#define PID_M_R_KD          0.0

// Control Timing Constants
#define CONTROL_UPDATE_RATE         10                                  // Hz
#define CONTROL_UPDATE_PERIOD       (1.0f / CONTROL_UPDATE_RATE)        // Seconds
#define CONTROL_UPDATE_PERIOD_US    (1'000'000 / CONTROL_UPDATE_RATE)   // Micro Seconds

// Encoder Constants
#define WHEEL_SEPERATION    0.020       // metres
#define WHEEL_RADIUS        0.0037      // metres
#define PULSE_PER_REV       256         

// Motor Constants
#define MOTOR_PWM_FREQ      10'000

// Bluetooth HM10 module default config constants
#define BT_BUFFER_SIZE      20
#define BT_BAUD_RATE        9600

// Maths constant
#define PI                  3.14159