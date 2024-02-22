#include "mbed.h"
#include "QEI.h"

#include "constants.h"
#include "pin_assignments.h"
#include "bluetooth.h"
#include "motor.h"
#include "encoder.h"
#include "vector_processor.h"
#include "PID.h"
#include "motor_driver_board.h"


typedef enum
{
    square_mode_start,
    square_mode_running,
    PID_test,
    inactive,
} Buggy_states;


DigitalOut LED(LED_PIN);                // Debug LED set
Serial pc(USBTX, USBRX, 115200);        // set up serial comm with pc
Bluetooth bt(BT_TX_PIN, BT_RX_PIN);     
MotorDriverBoard driver_board(DRIVER_ENABLE_PIN, true);
Timer global_timer;                     // set up global program timer
Ticker control_ticker;

int main_loop_counter = 0;                  // just for fun (not important)
int last_loop_time_us = 0;                  // stores the previous loop time
Buggy_states buggy_state = inactive;        // stores buggy states when performing actions

Motor motor_left(MOTORL_PWM_PIN, MOTORL_DIRECTION_PIN, MOTORL_BIPOLAR_PIN);
Motor motor_right(MOTORR_PWM_PIN , MOTORR_DIRECTION_PIN, MOTORR_BIPOLAR_PIN);

Encoder encoder_left(MOTORL_CHA_PIN, MOTORL_CHB_PIN);
Encoder encoder_right(MOTORR_CHA_PIN, MOTORR_CHB_PIN);

VectorProcessor vp;


PID PID_motor_left( 
    PID_M_L_KP, 
    PID_M_L_KI,
    PID_M_L_KD,
    PID_M_TAU,
    PID_M_MIN_OUT,
    PID_M_MAX_OUT,
    PID_M_MIN_INT,
    PID_M_MAX_INT
);

PID PID_motor_right(
    PID_M_R_KP, 
    PID_M_R_KI,
    PID_M_R_KD,
    PID_M_TAU,
    PID_M_MIN_OUT,
    PID_M_MAX_OUT,
    PID_M_MIN_INT,
    PID_M_MAX_INT
);


void control_update_ISR(void)
{
    encoder_left.update();
    encoder_right.update();
    vp.update(
        encoder_left.get_tick_count(), 
        encoder_right.get_tick_count());

    // PID Calculations here
    PID_motor_left.update(vp.get_left_set_speed(), encoder_left.get_speed());
    PID_motor_right.update(vp.get_right_set_speed(), encoder_right.get_speed());

    // Apply PID output
    if (buggy_state != inactive)
    {
        motor_left.set_duty_cycle(PID_motor_left.get_output());
        motor_right.set_duty_cycle(PID_motor_right.get_output());
    }
}



int main()
{
    while (!bt.is_ready()) {};          // while bluetooth not ready, loop and do nothing

    control_ticker.attach_us(&control_update_ISR, CONTROL_UPDATE_PERIOD_US);        // Starts the control ISR update ticker
    global_timer.start();                                                           // Starts the global program timer


    while (1)
    {
        /* --- START OF BLUETOOTH COMMAND HANDLING --- */
        if (bt.data_recieved_complete()) 
        {
            if (bt.parse_data())
            {
                switch(bt.cmd_type)
                {
                    case bt.get:
                        bt.set_send_once(true);
                        break;
                    case bt.set:
                        switch (bt.data_type)
                        {
                            case bt.pwm_duty:               // P
                                switch (bt.obj_type)
                                {
                                    case bt.motor_left:
                                        motor_left.set_duty_cycle(bt.data1);
                                        break;
                                    case bt.motor_right:
                                        motor_right.set_duty_cycle(bt.data1);
                                        break;
                                    default:
                                        break;
                                }
                                break;
                            case bt.speed:
                                vp.set_set_velocity(bt.data1); /// temporary to debug pid
                                break;
                            default:
                                break;
                        }
                        break;
                    case bt.execute:
                        switch (bt.exec_type)
                        {
                            case bt.stop:
                                // stop evrything
                                motor_left.set_duty_cycle(0.0);
                                motor_right.set_duty_cycle(0.0);
                                buggy_state = inactive;
                                break;
                            case bt.uturn:
                                // enable uturn code
                                buggy_state = PID_test; // temporary to test PID
                                break;
                            case bt.encoder_test:
                                motor_left.set_duty_cycle(0.0);
                                motor_right.set_duty_cycle(0.0);
                                bt.set_continous(true);
                                bt.data_type = bt.ticks_cumulative;
                                bt.obj_type = bt.motor_both;
                                break;
                            case bt.motor_pwm_test:
                                motor_left.set_duty_cycle(0.0);
                                motor_right.set_duty_cycle(0.0);
                                bt.set_continous(true);
                                bt.data_type = bt.pwm_duty;
                                bt.obj_type = bt.motor_both;
                                break;
                            case bt.square_test:
                                // enable move a in a square code
                                buggy_state = square_mode_start;
                                break;
                            case bt.toggle_led_test:
                                LED = !LED;
                                break;
                            default:
                                break;
                        }
                        break;
                    default:
                        break;
                }
            }
            else
            {
                bt.send_fstring("Invalid Command", bt.get_data());
            }
            bt.reset_rx_buffer();
        }

        // Handling sending data through BT 
        if (bt.is_continous() || bt.is_send_once())
        {   
            switch (bt.data_type)
            {
                case bt.pwm_duty:               // P
                    switch (bt.obj_type)
                    {
                        case bt.motor_left:
                            bt.send_fstring("DC L: %f", motor_left.get_duty_cycle());
                            break;
                        case bt.motor_right:
                            bt.send_fstring("DC R: %f", motor_right.get_duty_cycle());
                            break;
                        case bt.motor_both:
                            bt.send_fstring("DC L:%.3f/ R:%.3f", motor_left.get_duty_cycle(), motor_right.get_duty_cycle());
                            break;
                        default:
                            break;
                    }
                    break;
                case bt.ticks_cumulative:       // T
                    switch (bt.obj_type)
                    {
                        case bt.motor_left:
                            bt.send_fstring("Ticks L: %d", encoder_left.get_tick_count());
                            break;
                        case bt.motor_right:
                            bt.send_fstring("Ticks R: %d", encoder_right.get_tick_count());
                            break;
                        case bt.motor_both:
                            bt.send_fstring("L:%7d R:%7d", encoder_left.get_tick_count(), encoder_right.get_tick_count());
                            break;
                        default:
                            break;
                    }
                    break;
                case bt.speed:                  // V
                    switch (bt.obj_type)
                    {
                        case bt.motor_left:
                            bt.send_fstring("Speed L: %f", encoder_left.get_speed());
                            break;
                        case bt.motor_right:
                            bt.send_fstring("Speed R: %f", encoder_right.get_speed());
                            break;
                        case bt.motor_both:
                            bt.send_fstring("S L:%.3f/ R:%.3f", encoder_left.get_speed(), encoder_right.get_speed());
                            break;
                        default:
                            break;
                    }
                    break;
                case bt.gains_PID:              // G
                case bt.current_usage:          // C
                    bt.send_fstring("Feature WIP");
                    break;
                case bt.runtime:                // R
                    bt.send_fstring("Runtime: %f", global_timer.read());
                    break;
                case bt.loop_time:              // X
                    bt.send_fstring("Loop: %dus", global_timer.read_us() - last_loop_time_us);
                    break;
                case bt.loop_count:             // Y
                    bt.send_fstring("Loop no: %d", main_loop_counter);
                    break;
                default:
                    break;
            }
            bt.set_send_once(false); 
        }
        /* ---  END OF BLUETOOTH COMMAND HANDLING  --- */


        /* --- START OF BUGGY ACTIONS/STATE LOGIC CODE --- */ 
        switch (buggy_state) 
        {   
            case square_mode_start:
                vp.reset_distance_travelled();
                encoder_left.reset();
                encoder_right.reset();
                buggy_state = square_mode_running;
            case square_mode_running:
                // different distances change the set speed and angle
                break;
            case PID_test:
                break;
            default:
                break;
        }    
        /* ---  END OF BUGGY ACTIONS/STATE LOGIC CODE  --- */ 


        // Bunch of debug code:

        pc.printf("Left Encoder Pulse Count: %d \n", encoder_left.get_tick_count());
        pc.printf("Right Encoder Pulse Count: %d \n", encoder_right.get_tick_count());
        // float freq_l = encoder_left.get_freq(global_timer.read_us());
        // float freq_r = encoder_right.get_freq(global_timer.read_us());
        // pc.printf("Left Encoder Freq: %.2f \n", encoder_left.get_tick_count());
        // pc.printf("Right Encoder Freq: %.2f \n", freq_r);

        // float angle = vp.get_angle(freq_l, freq_r, global_timer.read_us());
        // total_angle += angle;
        // pc.printf("Angle Calculated: %.2f Degrees \n", angle);
        // pc.printf("Cumulative Angle: %.2f Degrees \n \n", total_angle);

        // motor_left.set_duty_cycle(0.01);
        // motor_right.set_duty_cycle(0.01);


        /*      ARTIFICIAL DELAY FOR DEBUGGING:    */
        wait_us(1'000'00);


        /*       END OF LOOP      */
        pc.printf("Loop Time: %d us / %d \n \n", global_timer.read_us() - last_loop_time_us, global_timer.read_us());
        last_loop_time_us = global_timer.read_us();
        main_loop_counter++;
        
    }
}
