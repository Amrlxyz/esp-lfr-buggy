#pragma once

// Debug LED pin
#define LED_PIN                 LED1

// Motor 1 Pins
#define MOTORL_BIPOLAR_PIN      PA_6
#define MOTORL_DIRECTION_PIN    PA_7
#define MOTORL_PWM_PIN          PB_6

// Motor 2 Pins
#define MOTORR_BIPOLAR_PIN      PC_7
#define MOTORR_DIRECTION_PIN    PA_9
#define MOTORR_PWM_PIN          PA_8

// Other Motor Driver Board Pins
#define DRIVER_ENABLE_PIN       PB_10
#define DRIVER_MONITOR_PIN      PB_4

// Motor Encoder Channels Pins
#define MOTORL_CHA_PIN          PA_0
#define MOTORL_CHB_PIN          PA_1
#define MOTORR_CHA_PIN          PB_3
#define MOTORR_CHB_PIN          PA_10

// Bluetooth Pins
#define BT_TX_PIN               PA_11
#define BT_RX_PIN               PA_12

// Sensor Analog Input Pins
#define SENSOR3L_OUT_PIN         PB_2
#define SENSOR2L_OUT_PIN         PB_1
#define SENSOR1L_OUT_PIN         PB_15
#define SENSOR1R_OUT_PIN         PB_14
#define SENSOR2R_OUT_PIN         PB_13
#define SENSOR3R_OUT_PIN         PC_4

// Sensor Digital Output Control Pin
#define SENSOR3L_IN_PIN         PC_2
#define SENSOR2L_IN_PIN         PC_3 
#define SENSOR1L_IN_PIN         PA_4 
#define SENSOR1R_IN_PIN         PB_0 
#define SENSOR2R_IN_PIN         PC_1 
#define SENSOR3R_IN_PIN         PC_0 
