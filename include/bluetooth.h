#pragma once
#include "mbed.h"
#include "constants.h"

class Bluetooth
{
protected:

    volatile int rx_index;
    volatile bool data_complete;
    RawSerial bt_serial;
    char tx_buffer[BT_BUFFER_SIZE]; /// buffer to store transmit data
    char rx_buffer[BT_BUFFER_SIZE]; /// buffer to store recieved data

    void init(void);
    
public:

    /// Constructors, if baud rate not specified the default BT baud rate is used
    Bluetooth(PinName TX_pin, PinName RX_pin, int baud_rate);
    Bluetooth(PinName TX_pin, PinName RX_pin);


    void data_recieved_ISR(void);


    bool data_recieved_complete(void);


    void reset_rx_buffer(void);


    char* get_data(void);


    int process_data(void);


    void send_buffer(char* strsing);


    void send_fstring(const char* format, ...);


    bool writeable(void);
};

