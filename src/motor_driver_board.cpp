#include "mbed.h"
#include "motor_driver_board.h"
#include "constants.h"
#include "pin_assignments.h"

#include "ds2781.h"
#include "OneWire_Methods.h"


DigitalInOut one_wire_pin(DRIVER_MONITOR_PIN);


MotorDriverBoard::MotorDriverBoard(PinName enable_pin, PinName monitor_pin): board_enable(enable_pin)
{
    disable();
}


void MotorDriverBoard::enable(void)
{
    enable_state = true;
    board_enable.write(true);
}


void MotorDriverBoard::disable(void)
{
    enable_state = false;
    board_enable.write(false);
}


void MotorDriverBoard::update_measurements(void)
{
    VoltageReading = ReadVoltage();
    Voltage = VoltageReading * 0.00967;
    CurrentReading = ReadCurrent();
    Current = CurrentReading / 6400.0;
}

void MotorDriverBoard::update_enable(bool expression)
{
    enable_state = expression;
    board_enable.write(expression);
}

float MotorDriverBoard::get_voltage(void) 
{
    return Voltage;
}

float MotorDriverBoard::get_current(void)
{
    return Current;
}


bool MotorDriverBoard::get_enable_state(void)
{
    return enable_state;
}
