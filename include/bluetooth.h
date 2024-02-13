#pragma once
#include "mbed.h"
#include "constants.h"


class Bluetooth
{
protected:

    RawSerial bt_serial;            // creates the RawSerial object to connect with the bluetooth module 
    volatile int rx_index;          // keeps track of the next memory location to store the next char recieved
    volatile bool data_complete;    // true if the incoming data if fully recieved
    char tx_buffer[BT_BUFFER_SIZE]; // buffer to store transmit data
    char rx_buffer[BT_BUFFER_SIZE]; // buffer to store recieved data

    /* This function used to initialise the Bluetooth object */
    void init(void);
    
public:

    typedef enum
    {
        stop,                   // Perform a stop Immediately
        uturn,                  // Perform 180 degree turn
        get_main_loop_count,    // Sends back loop count
        get_run_time,           // Sends back program run time
        get_encoderR_pulses,    // Sends back right encoder counter value 
        get_encoderL_pulses,    // Sends back left encoder counter value 
        get_motorR_PID,         // to be implemented
        get_motorL_PID,         // to be implemented 
        get_sensor_PID,         // to be implemented
        // other BT commands
        // other BT commands
        invalid,                // invalid command
    } BluetoothCommand;         // Bluetooth command types

    /// Constructors, if baud rate not specified the default BT baud rate is used
    Bluetooth(PinName TX_pin, PinName RX_pin, int baud_rate);
    /// Constructors, if baud rate not specified the default BT baud rate is used
    Bluetooth(PinName TX_pin, PinName RX_pin);

    /*  Returns true if incoming data is fully recieved*/
    void data_recieved_ISR(void);

    /*  This ISR will run for every character recieved by the bluetooth module.
        Each time this ISR is ran one character is stored into rx_buffer. 
        rx_index stores the location of next memory location to store the next character */
    bool data_recieved_complete(void);

    /*  Resets the recieved data buffer for new incoming data.
        Ideally used after processing recieved data. 
        Note: newly recieved data will be lost if rx_buffer was not reset */ 
    void reset_rx_buffer(void);

    /* returns the raw data recieved as a character array*/
    char* get_data(void);

    /* parse the recieved data to bluetooth command */
    BluetoothCommand parse_data(void);

    /* sends character array */
    void send_buffer(char* char_arr);

    /* sends formatted string similar to printf() */
    void send_fstring(const char* format, ...);

    /* returns true if bluetooth module is ready */
    bool writeable(void);

};

