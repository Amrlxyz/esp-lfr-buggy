#include "mbed.h"
#include "constants.h"
#include "pin_assignments.h"
#include "bluetooth.h"

PwmOut LED(LED_PIN);                    // Debug LED set
Serial pc(USBTX, USBRX, 9600);          // set up serial comm with pc
Bluetooth bt(BT_TX_PIN, BT_RX_PIN);
Timer global_timer;                     // set up global program timer


int main_loop_counter = 0;      // just for fun (not important)
int last_loop_time_us = 0;      // stores the previous loop time


int main()
{
    LED.period(0.05);
    global_timer.start();           // Starts the global program timer

    while(!bt.writeable()) {};      // wait for the bluetooth to be ready


    while(1)
    {
        // Checks for recieved bluetooth commands
        if (bt.data_recieved_complete()) {
            
            switch (bt.parse_data())
            {
                case bt.uturn:
                    bt.send_fstring("Making a U-turn");
                    // U-turn code here
                    break;
                case bt.get_run_time:
                    bt.send_fstring("Time: %.2f s", global_timer.read());
                    break;
                case bt.set_value:
                    bt.send_fstring("Duty Cycle: %f", bt.float_data1);
                    LED.write(bt.float_data1);
                    break;
                case bt.get_encoderL_pulses:
                    bt.send_fstring("L Enc: %d", main_loop_counter);
                    break;
                case bt.get_encoderR_pulses:
                    bt.send_fstring("R Enc: %d", main_loop_counter);
                    break;
                // case others:
                default:
                    bt.send_fstring("Received: %s\n", bt.get_data());
                    break;
            }
            bt.reset_rx_buffer();
        }

        // simulate other part of code:
        wait_us(1'000'000); // 1 sec delay


        // End of loop
        pc.printf("Loop Time: %d us / %d \n", global_timer.read_us() - last_loop_time_us, global_timer.read_us());
        last_loop_time_us = global_timer.read_us();
        main_loop_counter++;
    }
}