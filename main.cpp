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
    straight_test,
    PID_test,
    inactive,
} Buggy_states;


/* Variable Declarations */
Buggy_states buggy_state = inactive;        // stores buggy states when performing actions
volatile bool serial_update = false;
int main_loop_counter = 0;                  // just for fun (not important)
int last_loop_time_us = 0;                  // stores the previous loop time
int loop_time_diff = 0;
int square_stages = 0;


/* Objects Declarations */
DigitalOut LED(LED_PIN);                    // Debug LED set
Serial pc(USBTX, USBRX, 115200);            // set up serial comm with pc
Bluetooth bt(BT_TX_PIN, BT_RX_PIN);     
MotorDriverBoard driver_board(DRIVER_ENABLE_PIN, true);
Timer global_timer;                         // set up global program timer
Ticker control_ticker;
Ticker serial_ticker;

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
    int curr_time = global_timer.read_us();

    /* Run all the update functions: */
    encoder_left.update();
    encoder_right.update();
    vp.update(
        encoder_left.get_tick_count(), 
        encoder_right.get_tick_count());

    /* PID Calculations: */
    PID_motor_left.update(vp.get_left_set_speed(), encoder_left.get_filtered_speed());
    PID_motor_right.update(vp.get_right_set_speed(), encoder_right.get_filtered_speed());

    // pc.printf("%.4f,%.4f\n", encoder_left.get_speed(), encoder_left.get_filtered_speed());
    pc.printf("%.4f\n", vp.get_cumulative_angle_deg());

    /* Apply PID output */
    if (buggy_state == PID_test)
    {
        motor_left.set_duty_cycle(PID_motor_left.get_output());
        motor_right.set_duty_cycle(PID_motor_right.get_output());

        // pc.printf("SetL:%.4f,SetR:%.4f,FiltL:%.4f,ErrL:%.4f,OutL:%.4f,OutR:%.4f\n", vp.get_left_set_speed(), vp.get_right_set_speed(), encoder_left.get_filtered_speed(), vp.get_left_set_speed() - encoder_left.get_filtered_speed(), PID_motor_left.get_output(), PID_motor_right.get_output());
        // pc.printf("R - Meas: %f, Out: %f\n", encoder_right.get_speed(), PID_motor_right.get_output());
    }

    // Measure ISR time - Ignore the confusing names for now
    loop_time_diff = global_timer.read_us() - curr_time;
}


void serial_update_ISR(void)
{
    // serial_update = true;
}


void stop_reset(void)
{
    motor_left.set_duty_cycle(0.0);
    motor_right.set_duty_cycle(0.0);
    bt.send_fstring("%d, %.4f, %.4f", square_stages, vp.get_cumulative_angle_deg(), vp.get_distance_travelled());
    wait_us(1'000'000);
    encoder_left.reset();
    encoder_right.reset();
    vp.reset_distance_travelled();
}


int main()
{
    while (!bt.is_ready()) {};          // while bluetooth not ready, loop and do nothing

    global_timer.start();                                                           // Starts the global program timer
    control_ticker.attach_us(&control_update_ISR, CONTROL_UPDATE_PERIOD_US);        // Starts the control ISR update ticker
    serial_ticker.attach(&serial_update_ISR, SERIAL_UPDATE_PERIOD);                 // Starts the control ISR update ticker

    float right_speed_offset = -0.02;

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
                            case bt.speed:                  // S
                                right_speed_offset = bt.data1;
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
                            case bt.straight_test:
                                // enable move a in a square code
                                buggy_state = straight_test;
                                break;
                            case bt.square_test:
                                // enable move a in a square code
                                buggy_state = square_mode_start;
                                break;
                            case bt.PID_test:
                                // test the pid implementation
                                PID_motor_left.reset();
                                PID_motor_right.reset();
                                vp.reset_PID_angle();
                                vp.set_set_angle(90);
                                vp.set_set_velocity(0.0);
                                buggy_state = PID_test;
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
        /* ---  END OF BLUETOOTH COMMAND HANDLING  --- */

        float straight_speed = 0.25;
        float turn_speed = 0.25;
        float distance_travelling = 0.3;
        float turning_angle = 89.0;

        /* --- START OF BUGGY ACTIONS/STATE LOGIC CODE --- */ 
        switch (buggy_state) 
        {   
            case square_mode_start:
                vp.reset_distance_travelled();
                encoder_left.reset();
                encoder_right.reset();
                buggy_state = square_mode_running;
            case square_mode_running:
                switch (square_stages)
                {
                    case 0:
                        motor_left.set_duty_cycle(straight_speed);
                        motor_right.set_duty_cycle(straight_speed + right_speed_offset);
                        square_stages++;
                        break;
                    case 1:
                    case 3:
                    case 5:
                    case 7:
                        if (vp.get_distance_travelled() >= distance_travelling) // wait to move 1m then, start turning right
                        {
                            stop_reset();
                            motor_left.set_duty_cycle(turn_speed);
                            motor_right.set_duty_cycle(-(straight_speed + right_speed_offset));
                            square_stages++;
                        }
                        break;
                    case 2:
                    case 4:
                    case 6:
                        if (vp.get_cumulative_angle_deg() >= turning_angle) // wait to turn 90 and start moving straight
                        {
                            stop_reset();
                            motor_left.set_duty_cycle(straight_speed);
                            motor_right.set_duty_cycle(straight_speed + right_speed_offset);
                            square_stages++;
                        }
                        break;
                    case 8:
                        if (vp.get_cumulative_angle_deg() >= turning_angle * 2) // wait to turn 180 and then move straihgt
                        {
                            stop_reset();
                            motor_left.set_duty_cycle(straight_speed);
                            motor_right.set_duty_cycle(straight_speed + right_speed_offset);
                            square_stages++;
                        }
                        break;
                    case 9:
                    case 11:
                    case 13:
                        if (vp.get_distance_travelled() >= distance_travelling) // wait to move 1m then, start turning left
                        {
                            stop_reset();
                            motor_left.set_duty_cycle(-turn_speed);
                            motor_right.set_duty_cycle(straight_speed + right_speed_offset);
                            square_stages++;
                        }
                        break;
                    case 10:
                    case 12:
                    case 14:
                        if (vp.get_cumulative_angle_deg() <= -turning_angle) // wait to turn -90 and start moving straight
                        {
                            stop_reset();
                            motor_left.set_duty_cycle(straight_speed);
                            motor_right.set_duty_cycle(straight_speed + right_speed_offset);
                            square_stages++;
                        }
                        break;
                    case 15:
                        if (vp.get_distance_travelled() >= distance_travelling)  // Stop
                        {
                            stop_reset();
                            motor_left.set_duty_cycle(0.0);
                            motor_right.set_duty_cycle(0.0);
                            square_stages = 0;
                            buggy_state = inactive;
                        }
                        break;
                    default:
                        break;
                }
                break;
            case straight_test:
                motor_left.set_duty_cycle(straight_speed);
                motor_right.set_duty_cycle(straight_speed + right_speed_offset);
                if (vp.get_distance_travelled() >= 1)
                {
                    stop_reset();
                    buggy_state = inactive;
                }
                break;
            case PID_test:
                if (vp.get_distance_travelled() >= 0.5)
                {
                    buggy_state = inactive;
                    stop_reset();
                }
                break;
            default:
                break;
        }    
        /* ---  END OF BUGGY ACTIONS/STATE LOGIC CODE  --- */ 

        
        /* --- START OF SERIAL UPDATE CODE --- */
        if (serial_update)
        {
            // Handling sending data through BT 
            if (bt.is_continous() || bt.is_send_once())
            {   
                switch (bt.data_type)
                {
                    case bt.pwm_duty:               // P
                        switch (bt.obj_type)
                        {
                            case bt.motor_left:
                                bt.send_fstring("DC L: %.2f", motor_left.get_duty_cycle());
                                break;
                            case bt.motor_right:
                                bt.send_fstring("DC R: %.2f", motor_right.get_duty_cycle());
                                break;
                            case bt.motor_both:
                                bt.send_fstring("DC L:%.2f/ R:%.2f", motor_left.get_duty_cycle(), motor_right.get_duty_cycle());
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
                        bt.send_fstring("ISR: %dus", loop_time_diff);
                        break;
                    case bt.loop_count:             // Y
                        bt.send_fstring("Loop no: %d", main_loop_counter);
                        break;
                    default:
                        break;
                }
                bt.set_send_once(false); 
            }

            // PC Serial Debugging code

            pc.printf("\n");
            pc.printf("\n");

            pc.printf("Square State: %d \n", square_stages);

            pc.printf("Left  Motor Duty Cycle: %.2f \n", motor_left.get_duty_cycle());
            pc.printf("Right Motor Duty Cycle: %.2f \n", motor_right.get_duty_cycle());

            pc.printf("Left  Encoder Tick Count: %d \n", encoder_left.get_tick_count());
            pc.printf("Right Encoder Tick Count: %d \n", encoder_right.get_tick_count());
            
            pc.printf("Left  Motor Speed (m/s): %.5f \n", encoder_left.get_speed());
            pc.printf("Right Motor Speed (m/s): %.5f \n", encoder_right.get_speed());

            pc.printf("\n");

            pc.printf("Cumulative Angle: %.3f Degrees \n", vp.get_cumulative_angle_deg());
            pc.printf("Distance Travelled: %.3f Metres \n", vp.get_distance_travelled());

            pc.printf("\n");

            pc.printf("Calculation ISR Time: %d us / Global Time: %d us \n", loop_time_diff, global_timer.read_us());

            serial_update = false;
        }
        /* ---  END OF SERIAL UPDATE CODE  --- */


        /*      ARTIFICIAL DELAY FOR DEBUGGING:    */
        // wait_us(1'000'00);


        /*       END OF LOOP      */
        // loop_time_diff = global_timer.read_us() - last_loop_time_us;
        last_loop_time_us = global_timer.read_us();
        main_loop_counter++;
    }
}
