#include "mbed.h"

#include "bluetooth.h"


Bluetooth::Bluetooth(PinName TX_pin, PinName RX_pin, int baud_rate): bt_serial(TX_pin, RX_pin, baud_rate) {init();};


void Bluetooth::init(void)
{
    /* This function used to initialise the Bluetooth object */
    bt_serial.attach(callback(this, &Bluetooth::data_recieved_ISR), Serial::RxIrq);
    rx_index = 0;
    data_complete = false;
}


void Bluetooth::data_recieved_ISR(void)
{
    /*  This ISR will run for every character recieved by the bluetooth module.
        Each time this ISR is ran one character is stored into rx_buffer. 
        rx_index stores the location of next memory location to store the next character */
    char c = bt_serial.getc();
    if (!data_complete)
    {
        rx_buffer[rx_index++] = c;
        if (c == '/' || rx_index == buffer_size)
        {
            data_complete = true;
        }
    }
}


bool Bluetooth::data_recieved_complete(void)
{   
    /* Returns true if incoming data is fully recieved*/
    return data_complete;
}


void Bluetooth::reset_rx_buffer(void)
{   
    /* Resets the rx_buffer. Ideally used after processing recieved data*/ 
    memset(rx_buffer, '\0', buffer_size);
    data_complete = false;
    rx_index = 0;
}


void Bluetooth::send_buffer(char* char_arr)
{   
    /*  sends the char array one by one */
    for(int i = 0; i < buffer_size - 1; i++)
    {
        char ch = char_arr[i];
        bt_serial.putc(ch);
        if (ch == '\0')
        {
            break;
        }
    }
    bt_serial.putc('\n');
}


void Bluetooth::send_fstring(const char* format, ...) 
{   
    /*  This one is a bit advanced but the macros used are common to be 
        used to pass printf arguments into a function */
    va_list args;               // standard macro
    va_start(args, format);     // standard macro
    vsnprintf(tx_buffer, buffer_size + 1, format, args);     // converts the format string input to a character array and stores into tx_buffer
    va_end(args);               // standard macro
    send_buffer(tx_buffer);     // sends the character array to be transmitted using bluetooth
    memset(tx_buffer, '\0', buffer_size);                    // reset the tx_buffer
}


bool Bluetooth::is_ready(void)
{   
    /* returns true if bluetooth module is ready */
    return bt_serial.writeable();
}


bool Bluetooth::is_continous(void)
{
    return continous_update;
}


char* Bluetooth::get_rx_buffer(void)
{   
    /* returns data recieved as a character array*/
    return rx_buffer;
}


bool Bluetooth::is_send_once(void)   
{
    return send_once;
}


void Bluetooth::set_send_once(bool status)
{
    send_once = status;
}


void Bluetooth::set_continous(bool status)
{
    continous_update = status; 
}



// bool Bluetooth::parse_data(void)
// {   
//     /*  This function reads the incoming data and checks for 
//         specific command format and returns the command type 
//         returns false if parsing failed     */

//     // command parsing -> to-do 
//     switch (rx_buffer[0]) 
//     {
//         case 'E': case 'e':
//             cmd_type = execute;
//             break;
//         case 'S': case 's':
//             cmd_type = set;
//             break;
//         case 'G': case 'g':
//             cmd_type = get;
//             break;
//         case 'C': case 'c':
//             cmd_type = continous;
//             continous_update = !continous_update;
//             return true;
//         default:
//             return false;
//     }

//     if (cmd_type == execute)
//     {
//         switch (rx_buffer[1])
//         {
//             case 'S':
//                 exec_type = stop;
//                 break;
//             case 'U':
//                 exec_type = uturn;
//                 break;
//             case 'E':
//                 exec_type = encoder_test;
//                 break;
//             case 'M':
//                 exec_type = motor_pwm_test;
//                 break;
//             case 'C':
//                 exec_type = straight_test;
//                 break;
//             case 'Q':
//                 exec_type = square_test;
//                 break;
//             case 'P':
//                 exec_type = PID_test;
//                 break;
//             case 'L':
//                 exec_type = toggle_led_test;
//                 break;
//             case 'F':
//                 exec_type = line_follow;
//                 break;
//             default:
//                 return false;
//         }
//     }
//     else
//     {
//         switch (rx_buffer[1])
//         {
//             case 'P':
//                 data_type = pwm_duty;
//                 break;
//             case 'T':
//                 data_type = ticks_cumulative;
//                 break;
//             case 'S':
//                 data_type = speed;
//                 break;
//             case 'G':
//                 data_type = gains_PID;
//                 break;
//             case 'C':
//                 data_type = current_usage;
//                 break;
//             case 'R':
//                 data_type = runtime;
//                 break;
//             case 'X':
//                 data_type = loop_time;
//                 break;
//             case 'Y':
//                 data_type = loop_count;
//                 break;
//             default:
//                 return false;
//         }
        
//         switch (rx_buffer[2]) 
//         {
//             case 'L':
//                 obj_type = motor_left;
//                 break;
//             case 'R':
//                 obj_type = motor_right;
//                 break;
//             case 'B':
//                 obj_type = motor_both;
//                 break;
//             case 'S':
//                 obj_type = sensor;
//                 break;
//             default:
//                 obj_type = no_obj;
//                 break;
//         }

//         if (cmd_type == get)
//         {
//             data_type_sent = data_type;
//             obj_type_sent = obj_type;
//         }
//     }

//     if (cmd_type == set)
//     {   
//         int data_amount = (data_type == gains_PID) ? 3 : 1;
//         if (sscanf(rx_buffer, "%*s %f %f %f", &data1, &data2, &data3) != data_amount)
//         {
//             return false;
//         }
//     } 

//     return true;
// }