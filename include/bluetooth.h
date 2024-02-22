#pragma once

#include "mbed.h"
#include "constants.h"


class Bluetooth
{
protected:

    RawSerial bt_serial;            // creates the RawSerial object to connect with the bluetooth module 
    bool continous_update;          // if this is true, sends data on each loop without bt commands.
    bool send_once;                 // true when get cmd is used
    volatile int rx_index;          // keeps track of the next memory location to store the next char recieved
    volatile bool data_complete;    // true if the incoming data if fully recieved
    char tx_buffer[BT_BUFFER_SIZE]; // buffer to store transmit data
    char rx_buffer[BT_BUFFER_SIZE]; // buffer to store recieved data

    /* This function used to initialise the Bluetooth object */
    void init(void);
    
public:

    // Possible bluetooth command types
    typedef enum
    {
        execute,                // E
        get,                    // G
        set,                    // S
        continous,              // C
        // invalid,
    } BluetoothCommandTypes;

    typedef enum
    {   
        stop,                   // S
        uturn,                  // U
        encoder_test,           // E
        motor_pwm_test,         // M
        square_test,            // Q
        toggle_led_test,        // L
    } BluetoothCommandExecTypes;

    typedef enum
    {
        pwm_duty,               // P
        ticks_cumulative,       // T
        speed,                  // S
        gains_PID,              // G
        current_usage,          // C
        runtime,                // R
        loop_time,              // X
        loop_count,             // Y
    } BluetoothCommandDataTypes;

    typedef enum
    {
        motor_left,         // L // PID, Encoder Ticks, Velocity
        motor_right,        // R // PID, Encoder Ticks, Velocity
        motor_both,         // B
        sensor,             // S
        no_obj,             // default case
    } BluetoothCommandObjects;

    BluetoothCommandTypes       cmd_type;
    BluetoothCommandExecTypes   exec_type;
    BluetoothCommandDataTypes   data_type;
    BluetoothCommandObjects     obj_type;
    float data1, data2, data3;                      // Stores float data from command
    // int   int_data1, int_data2, intdata_3;       // Stores int data from command


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

    /*  parse the recieved data to bluetooth command */
    bool parse_data(void);

    /*  Sends character array */
    void send_buffer(char* char_arr);

    /*  sends formatted string 
        Used the same way as printf() */
    void send_fstring(const char* format, ...);

    /*  returns true if bluetooth module is ready */
    bool is_ready(void);

    /*  returns true if continous update is enabled */
    bool is_continous(void);


    bool is_send_once(void);

    /*  returns the raw data recieved as a character array */
    char* get_data(void);
    

    void set_send_once(bool status);


    void set_continous(bool);
};


