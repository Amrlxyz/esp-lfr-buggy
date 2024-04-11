/**
 * @file bluetooth.h
 * @brief BLE HM-10 Library 
 * 
 */

#pragma once

#include "mbed.h"

/**
 * @brief BLE HM-10 Interface Class
 * 
 * Library to interface with the HM-10 Bluetooth Low Energy Module
 *  
 * BLE has a standard of 20 bytes maximum per packet so it cant be used like a normal UART serial and theres problems with buffering
 * 
 * So this class Utilises RawSerial to get and send individual characters
 * 
 * Has functions that sends and recieves individual characters in a loop (keep in mind 20 bytes limit)
 * 
 * "Continous update" and "send once" property are stored in the class but handled externally 
 * 
 */
class Bluetooth
{
protected:

    const static int buffer_size = 20;  ///< Number of bits per packet (20)

    RawSerial bt_serial;                ///< creates the RawSerial object to connect with the bluetooth module 
    bool continous_update;              ///< if this is true, sends data on each loop without bt commands.
    bool send_once;                     ///< true when get cmd is used
    volatile int rx_index;              ///< keeps track of the next memory location to store the next char recieved
    volatile bool data_complete;        ///< true if the incoming data if fully recieved
    char tx_buffer[buffer_size + 1];    ///< buffer to store transmit data
    char rx_buffer[buffer_size + 1];    ///< buffer to store recieved data
    
    /* This function used to initialise the Bluetooth object */
    void init(void);
    

public:

    /**
     * @brief Construct a new Bluetooth object
     * 
     * @param TX_pin TX pin on the MCU, but RX pin on the BT Module
     * @param RX_pin RX pin on the MCU, but TX pin on the BT Module
     * @param baud_rate The bluetooth module baudrate
     */
    Bluetooth(PinName TX_pin, PinName RX_pin, int baud_rate); 

    /**
     * @brief Returns true if incoming data is fully recieved
     * 
     */
    bool data_recieved_complete(void);

    /**
     * @brief ISR that runs for every character recieved.
     * 
     * This ISR will run for every character recieved by the bluetooth module.
     * 
     * Each time this ISR is ran one character is stored into rx_buffer.
     *  
     * rx_index stores the location of next memory location to store the next character
     * 
     */ 
    void data_recieved_ISR(void);

    /**
     * @brief Resets the recieved data buffer for new incoming data.
     * 
     * Ideally used after processing recieved data. 
     * 
     * Note: newly recieved data will be lost if rx_buffer was not reset
     */
    void reset_rx_buffer(void);

    /**
     * @brief Sends character array to the bluetooth
     * 
     * @param char_arr pointer to the character array
     */
    void send_buffer(char* char_arr);

    
    /**
     * @brief Sends formatted string to the bluetooth 
     * 
     * Used the same way as printf()
     */
    void send_fstring(const char* format, ...);

    /**
     * @brief returns true if bluetooth module is ready
     * 
     * */
    bool is_ready(void);

    /**
     * @brief returns true if continous update is enabled
     * 
     */
    bool is_continous(void);

    /**
     * @brief returns true if send once is enabled
     * 
     */
    bool is_send_once(void);

    /**
     * @brief returns the raw data recieved as a character array
     * 
     * @return pointer to the rx buffer (character array)
     */
    char* get_rx_buffer(void);
    

    /**
     * @brief Set the send once property to the bool value passed
     * 
     * @param status bool value to be applied
     */
    void set_send_once(bool status);

    /**
     * @brief Set the continous property to the bool value passed
     * 
     * @param status bool value to be applied
     */
    void set_continous(bool status);
};



    // // Possible bluetooth command types
    // typedef enum
    // {
    //     execute = 'E',                // E
    //     get,                    // G
    //     set,                    // S
    //     continous,              // C
    //     // invalid,
    // } BluetoothCommandTypes;

    // typedef enum
    // {   
    //     stop,                   // S
    //     uturn,                  // U
    //     encoder_test,           // E
    //     motor_pwm_test,         // M
    //     straight_test,          // C
    //     square_test,            // Q
    //     PID_test,               // P
    //     toggle_led_test,        // L
    //     line_follow,            // F
    // } BluetoothCommandExecTypes;

    // typedef enum
    // {
    //     pwm_duty,               // P
    //     ticks_cumulative,       // T
    //     speed,                  // S
    //     gains_PID,              // G
    //     current_usage,          // C
    //     runtime,                // R
    //     loop_time,              // X
    //     loop_count,             // Y
    // } BluetoothCommandDataTypes;

    // typedef enum
    // {
    //     motor_left,         // L // PID, Encoder Ticks, Velocity
    //     motor_right,        // R // PID, Encoder Ticks, Velocity
    //     motor_both,         // B
    //     sensor,             // S
    //     no_obj,             // default case
    // } BluetoothCommandObjects;

    // BluetoothCommandTypes       cmd_type;
    // BluetoothCommandExecTypes   exec_type;
    // BluetoothCommandDataTypes   data_type;
    // BluetoothCommandObjects     obj_type;
    
    // BluetoothCommandDataTypes   data_type_sent;
    // BluetoothCommandObjects     obj_type_sent; 
    
    // float data1, data2, data3;                      // Stores float data from command