#include "mbed.h"
#include "constants.h"
#include "pin_assignments.h"
#include "bluetooth.h"


Serial pc(USBTX, USBRX, 9600);


int main()
{
    int counter = 0;
    Bluetooth bt(PA_11, PA_12); 

    while (!bt.writeable()) {};

    while(1)
    {
        if (bt.data_recieved_complete()) {
            pc.printf("Received: %s\n", bt.get_data());
            bt.reset_rx_buffer();
        }

        wait_us(1000000);
        bt.send_fstring("loop number: %d \n", counter);
        counter++;
    }
}