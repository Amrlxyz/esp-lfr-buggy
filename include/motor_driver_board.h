#pragma once

#include "mbed.h"


class MotorDriverBoard
{
protected:
    
    DigitalOut board_enable;
    bool enable_state;

    DigitalInOut one_wire_pin;
    int VoltageReading, CurrentReading;
    float Voltage, Current;

public:

    MotorDriverBoard(PinName enable_pin, PinName monitor_pin);

    void update_measurements(void);

    void update_enable(bool expression);

    void enable(void);
    
    void disable(void);

    float get_voltage(void);

    float get_current(void);

    bool get_enable_state(void);
};
