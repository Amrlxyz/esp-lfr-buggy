#pragma once

// Bluetooth HM10 module default config constants
#define BT_BUFFER_SIZE      20
#define BT_BAUD_RATE        9600

// Motor Constants
#define MOTOR_PWM_FREQ      10'000

// Encoder Constants
#define WHEEL_SEPERATION    0.020       // metres
#define WHEEL_RADIUS        0.0037      // metres
#define PULSE_PER_REV       256         

// Control Timing Constants
#define CONTROL_UPDATE_RATE         10                                  // Hz
#define CONTROL_UPDATE_PERIOD_US    1'000'000 / CONTROL_UPDATE_RATE     // Micro Seconds



