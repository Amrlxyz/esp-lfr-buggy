#pragma once

#include "mbed.h"


class MotorDriverBoard
{
protected:
    
    DigitalOut board_enable;
    bool enable_state;

public:

    MotorDriverBoard(PinName enable_pin);
    MotorDriverBoard(PinName enable_pin, bool state);

    void enable(void);
    
    void disable(void);

    bool get_enable_state(void);
};
