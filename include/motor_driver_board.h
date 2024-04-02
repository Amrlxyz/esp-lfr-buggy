#pragma once

#include "mbed.h"

#include "ds2781.h"
#include "OneWire_Methods.h"


class MotorDriverBoard
{
protected:
    
    DigitalOut board_enable;
    bool enable_state;

    int VoltageReading, CurrentReading;
    float Voltage, Current;

public:

    MotorDriverBoard(PinName enable_pin, PinName monitor_pin);

    void update_measurements(void);

    void set_enable(bool expression);

    void enable(void);
    
    void disable(void);

    float get_voltage(void);

    float get_current(void);

    bool get_enable_state(void);
};
