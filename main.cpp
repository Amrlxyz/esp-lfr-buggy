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


/* BUGGY STATES TYPE */
typedef enum
{
    square_mode,
    straight_test,
    PID_test,
    task_test,
    task_test_inactive,
    inactive,               // motor driver off
} Buggy_states;


/* GLOBAL VARIBLES DECLARATIONS */
Buggy_states buggy_state = inactive;        // stores buggy states when performing actions
volatile bool pc_serial_update = false;
volatile bool bt_serial_update = false;
int ISR_exec_time = 0;
int square_stages = 0;


/* OBJECTS DECLARATIONS */
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
PID PID_motor_left(PID_M_L_KP, PID_M_L_KI, PID_M_L_KD, PID_M_TAU, PID_M_MIN_OUT, PID_M_MAX_OUT, PID_M_MIN_INT, PID_M_MAX_INT);
PID PID_motor_right(PID_M_R_KP, PID_M_R_KI, PID_M_R_KD, PID_M_TAU, PID_M_MIN_OUT, PID_M_MAX_OUT, PID_M_MIN_INT, PID_M_MAX_INT);
VectorProcessor vp;


/* ISR FUNCTIONS */
void control_update_ISR(void)
{   
    // Get the current time to measure the execution time
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
    // pc.printf("%.4f\n", vp.get_cumulative_angle_deg());


    /* Apply PID output */
    if (buggy_state == PID_test || buggy_state == square_mode)
    {
        motor_left.set_duty_cycle(PID_motor_left.get_output());
        motor_right.set_duty_cycle(PID_motor_right.get_output());

        // pc.printf("SetL:%.4f,SetR:%.4f,FiltL:%.4f,ErrL:%.4f,OutL:%.4f,OutR:%.4f\n", vp.get_left_set_speed(), vp.get_right_set_speed(), encoder_left.get_filtered_speed(), vp.get_left_set_speed() - encoder_left.get_filtered_speed(), PID_motor_left.get_output(), PID_motor_right.get_output());
        // pc.printf("R - Meas: %f, Out: %f\n", encoder_right.get_speed(), PID_motor_right.get_output());
    }

    // Measure control ISR execution time
    ISR_exec_time = global_timer.read_us() - curr_time;
}


void serial_update_ISR(void)
{
    pc_serial_update = true;
    bt_serial_update = true;
}


/* OTHER FUNCTIONS */
void square_stage_end(void)
{
    bt.send_fstring("%d, %.4f, %.4f", square_stages, vp.get_cumulative_angle_deg(), vp.get_distance_travelled());
}


void reset_everything(void)
{
    encoder_left.reset();
    encoder_right.reset();
    PID_motor_left.reset();
    PID_motor_right.reset();
    vp.reset_all();
}


void stop_motors(void)
{
    motor_left.set_duty_cycle(0.0);
    motor_right.set_duty_cycle(0.0);
}


void update_motor_board(void)
{
    // Disabling the driver board if in inactive state
    if (buggy_state == inactive || buggy_state == task_test_inactive) 
    {
        driver_board.disable();
    } 
    else 
    {
        driver_board.enable();
    }
}


/* MAIN FUNCTION */
int main()
{
    while (!bt.is_ready()) {};          // while bluetooth not ready, loop and do nothing

    driver_board.disable();
    global_timer.start();                                                           // Starts the global program timer
    control_ticker.attach_us(&control_update_ISR, CONTROL_UPDATE_PERIOD_US);        // Starts the control ISR update ticker
    serial_ticker.attach(&serial_update_ISR, SERIAL_UPDATE_PERIOD);                 // Starts the control ISR update ticker

    // Straight test variables
    float straight_right_speed_offset = -0.02;
    float straight_speed = 0.25;

    // Square task variables
    float square_angle_set = 0.0;
    float square_distance_set = 0.0;
    float square_velocity_set = 0.4;
    float square_turning_left_angle = 103;  // increase to turn more
    float square_turning_right_angle = 94;  // decrease to turn more 
    float square_distance = 1.01;

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
                                straight_right_speed_offset = bt.data1;
                                break;
                            default:
                                break;
                        }
                        break;
                    case bt.execute:
                        switch (bt.exec_type)
                        {
                            case bt.stop:
                                // EMERGENCY STOP
                                stop_motors();
                                buggy_state = inactive;
                                break;
                            case bt.uturn:
                                buggy_state = inactive; // WIP
                                break;
                            case bt.encoder_test:
                                stop_motors();
                                bt.set_continous(true);
                                bt.data_type = bt.ticks_cumulative;
                                bt.obj_type = bt.motor_both;
                                buggy_state = task_test;
                                break;
                            case bt.motor_pwm_test:
                                stop_motors();
                                bt.set_continous(true);
                                bt.data_type = bt.pwm_duty;
                                bt.obj_type = bt.motor_both;
                                buggy_state = task_test_inactive;
                                break;
                            case bt.straight_test:
                                reset_everything();
                                buggy_state = straight_test;
                                break;
                            case bt.square_test:
                                reset_everything();
                                buggy_state = square_mode;
                                break;
                            case bt.PID_test:
                                reset_everything();
                                vp.set_set_angle(90);
                                vp.set_set_velocity(0.0);
                                buggy_state = PID_test;
                                break;
                            case bt.toggle_led_test:
                                LED = !LED;
                                buggy_state = task_test_inactive;
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


        /* --- START OF BUGGY ACTIONS/STATE LOGIC CODE --- */ 
        update_motor_board();
        switch (buggy_state) 
        {   
            case square_mode:
                switch (square_stages)
                {
                    case 0:
                        square_distance_set += square_distance;
                        vp.set_set_angle(square_angle_set);
                        vp.set_set_velocity(square_velocity_set);
                        square_stages++;
                        square_stage_end();
                        break;
                    case 7:
                    case 1:
                    case 3:
                    case 5:
                        if (vp.get_distance_travelled() >= square_distance_set) // wait to move 1m then, start turning right
                        {
                            if (square_stages == 7)
                            {
                                square_angle_set += square_turning_right_angle;
                            }
                            square_angle_set += square_turning_right_angle;
                            vp.set_set_angle(square_angle_set);
                            vp.set_set_velocity(0);
                            square_stages++;
                            square_stage_end();
                        }
                        break;
                    case 2:
                    case 4:
                    case 6:
                    case 8:
                        if (vp.get_cumulative_angle_deg() >= square_angle_set) // wait to turn 90 and start moving straight
                        {
                            square_distance_set += square_distance;
                            vp.set_set_angle(square_angle_set);
                            vp.set_set_velocity(square_velocity_set);
                            square_stages++;
                            square_stage_end();
                        }
                        break;
                    case 9:
                    case 11:
                    case 13:
                        if (vp.get_distance_travelled() >= square_distance_set) // wait to move 1m then, start turning left
                        {
                            square_angle_set -= square_turning_left_angle;
                            vp.set_set_angle(square_angle_set);
                            vp.set_set_velocity(0);
                            square_stages++;
                            square_stage_end();
                        }
                        break;
                    case 10:
                    case 12:
                    case 14:
                        if (vp.get_cumulative_angle_deg() <= square_angle_set) // wait to turn -90 and start moving straight
                        {
                            square_distance_set += square_distance;
                            vp.set_set_angle(square_angle_set);
                            vp.set_set_velocity(square_velocity_set);
                            square_stages++;
                            square_stage_end();
                        }
                        break;
                    case 15:
                        if (vp.get_distance_travelled() >= square_distance_set)  // Stop
                        {
                            motor_left.set_duty_cycle(0.0);
                            motor_right.set_duty_cycle(0.0);
                            square_stages = 0;
                            buggy_state = inactive;
                            square_stage_end();
                        }
                        break;
                    default:
                        break;
                }
                break;
            case straight_test:
                motor_left.set_duty_cycle(straight_speed);
                motor_right.set_duty_cycle(straight_speed + straight_right_speed_offset);
                if (vp.get_distance_travelled() >= 1)
                {
                    buggy_state = inactive;
                    motor_left.set_duty_cycle(0.0);
                    motor_right.set_duty_cycle(0.0);
                    reset_everything();
                }
                break;
            case PID_test:
                if (vp.get_distance_travelled() >= 0.5)
                {
                    buggy_state = inactive;
                    reset_everything();
                }
                break;
            default:
                break;
        }    
        /* ---  END OF BUGGY ACTIONS/STATE LOGIC CODE  --- */ 

        
        /* --- START OF SERIAL UPDATE CODE --- */
        if (bt_serial_update)
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
                        bt.send_fstring("ISR: %dus", ISR_exec_time);
                        break;
                    case bt.loop_count:             // Y
                        bt.send_fstring("removed feature");
                        break;
                    default:
                        break;
                }
                bt.set_send_once(false); 
            }
            bt_serial_update = false;
        }

        if (pc_serial_update)
        {
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

            pc.printf("Calculation ISR Time: %d us / Global Time: %d us \n", ISR_exec_time, global_timer.read_us());

            pc_serial_update = false;
        }
        /* ---  END OF SERIAL UPDATE CODE  --- */


        /*       END OF LOOP      */
    }
}
