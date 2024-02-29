#include "mbed.h"
#include "constants.h"
#include "motor_driver_board.h"


MotorDriverBoard::MotorDriverBoard(PinName enable_pin): board_enable(enable_pin) {};
MotorDriverBoard::MotorDriverBoard(PinName enable_pin, bool state): board_enable(enable_pin)
{
    enable_state = state;
    board_enable.write(state);
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

bool MotorDriverBoard::get_enable_state(void)
{
    return enable_state;
}
