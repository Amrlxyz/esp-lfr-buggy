#include "mbed.h"
#include "bluetooth.h"
#include "constants.h"


Bluetooth::Bluetooth(PinName TX_pin, PinName RX_pin, int baud_rate): bt_serial(TX_pin, RX_pin, baud_rate) {init();};
Bluetooth::Bluetooth(PinName TX_pin, PinName RX_pin): bt_serial(TX_pin, RX_pin, BT_BAUD_RATE) {init();};


void Bluetooth::init(void)
{
    bt_serial.attach(callback(this, &Bluetooth::data_recieved_ISR), Serial::RxIrq);
    rx_index = 0;
    data_complete = false;
}


void Bluetooth::data_recieved_ISR(void)
{
    char c = bt_serial.getc();
    if (!data_complete)
    {
        rx_buffer[rx_index++] = c;
        if (c == '-' || rx_index == BT_BUFFER_SIZE)
        {
            data_complete = true;
        }
    }
}


bool Bluetooth::data_recieved_complete(void)
{
    return data_complete;
}


void Bluetooth::reset_rx_buffer(void)
{
    memset(rx_buffer, '\0', BT_BUFFER_SIZE);
    data_complete = false;
    rx_index = 0;
}


char* Bluetooth::get_data()
{
    return rx_buffer;
}


int Bluetooth::process_data()
{
    return 0;
}


void Bluetooth::send_buffer(char* strsing)
{
    for(int i = 0; i < 20; i++)
    {
    bt_serial.putc(strsing[i]); 
    }
}


void Bluetooth::send_fstring(const char* format, ...) {
    va_list args;
    va_start(args, format);
    vsnprintf(tx_buffer, BT_BUFFER_SIZE, format, args);
    va_end(args);
    send_buffer(tx_buffer);
    memset(tx_buffer, '\0', BT_BUFFER_SIZE);
}


bool Bluetooth::writeable(void)
{
    return bt_serial.writeable();
}
