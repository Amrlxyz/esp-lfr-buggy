#include "mbed.h"
#include "QEI.h"

#include "constants.h"
#include "pin_assignments.h"
#include "bluetooth.h"
#include "motor.h"
#include "PID.h"
#include "motor_driver_board.h"
#include "sensor_array.h"


/* BUGGY STATES TYPE */
enum
{
    square_mode,
    straight_test,
    PID_test,
    line_test,
    line_follow,
    task_test,
    task_test_inactive,
    inactive,               // motor driver off
} buggy_state = inactive;   // stores buggy states when performing actions


/* BUGGY STATUS */
struct
{
    float set_velocity;
    float set_angle;

    float left_set_speed;
    float right_set_speed;

    float cumulative_angle_deg;
    float distance_travelled;
} buggy_status = {0};


struct
{
    float set_angle;
    float set_distance;
    int stage;
} square_task = {0};


/* GLOBAL VARIBLES DECLARATIONS */
volatile bool pc_serial_update = false;
volatile bool bt_serial_update = false;
int ISR_exec_time = 0;
int loop_exec_time = 0;



/* OBJECTS DECLARATIONS */
DigitalOut LED(LED_PIN);                    // Debug LED set
Serial pc(USBTX, USBRX, 115200);            // set up serial comm with pc
Timer global_timer;                         // set up global program timer
Ticker control_ticker;
Ticker serial_ticker;

Bluetooth bt(BT_TX_PIN, BT_RX_PIN, BT_BAUD_RATE);     
MotorDriverBoard driver_board(DRIVER_ENABLE_PIN, DRIVER_MONITOR_PIN);
SensorArray sensor_array(SENSOR0_IN_PIN, SENSOR1_IN_PIN, SENSOR2_IN_PIN, SENSOR3_IN_PIN, SENSOR4_IN_PIN, SENSOR5_IN_PIN,
                        SENSOR0_OUT_PIN, SENSOR1_OUT_PIN, SENSOR2_OUT_PIN, SENSOR3_OUT_PIN, SENSOR4_OUT_PIN, SENSOR5_OUT_PIN, SENS_SAMPLE_COUNT);
Motor motor_left(MOTORL_PWM_PIN, MOTORL_DIRECTION_PIN, MOTORL_BIPOLAR_PIN, MOTORL_CHA_PIN, MOTORL_CHB_PIN, PULSE_PER_REV, MOTOR_PWM_FREQ, CONTROL_UPDATE_RATE, LP_SPEED_A0, LP_SPEED_B0, LP_SPEED_B1, WHEEL_RADIUS);
Motor motor_right(MOTORR_PWM_PIN , MOTORR_DIRECTION_PIN, MOTORR_BIPOLAR_PIN, MOTORR_CHA_PIN, MOTORR_CHB_PIN, PULSE_PER_REV, MOTOR_PWM_FREQ, CONTROL_UPDATE_RATE, LP_SPEED_A0, LP_SPEED_B0, LP_SPEED_B1, WHEEL_RADIUS);
PID PID_motor_left(PID_M_L_KP, PID_M_L_KI, PID_M_L_KD, PID_M_TAU, PID_M_MIN_OUT, PID_M_MAX_OUT, PID_M_MIN_INT, PID_M_MAX_INT);
PID PID_motor_right(PID_M_R_KP, PID_M_R_KI, PID_M_R_KD, PID_M_TAU, PID_M_MIN_OUT, PID_M_MAX_OUT, PID_M_MIN_INT, PID_M_MAX_INT);
PID PID_angle(PID_A_KP, PID_A_KI, PID_A_KD, PID_A_TAU, PID_A_MIN_OUT, PID_A_MAX_OUT, PID_A_MIN_INT, PID_A_MAX_INT);


/* HELPER FUNCTIONS */
void stop_motors(void)
{
    motor_left.set_duty_cycle(0.0);
    motor_right.set_duty_cycle(0.0);
}


void update_buggy_status(int tick_count_left, int tick_count_right)
{
    // Angle calculations
    buggy_status.cumulative_angle_deg = (float) (360 * WHEEL_RADIUS) * (tick_count_left - tick_count_right) / (WHEEL_SEPERATION * 4 * PULSE_PER_REV);

    // distance travelled calculation
    buggy_status.distance_travelled = ((float) (tick_count_left + tick_count_right) / (2 * PULSE_PER_REV * 4)) * 2 * PI * WHEEL_RADIUS;
}


void reset_everything(void)
{
    motor_left.reset();
    motor_right.reset();
    PID_motor_left.reset();
    PID_motor_right.reset();

    buggy_status = {0};
}


/* ISR FUNCTIONS */
void control_update_ISR(void)
{   
    // Get the current time to measure the execution time
    int curr_time = global_timer.read_us();

    /* Run all the update functions: */
    sensor_array.update();
    motor_left.update();
    motor_right.update();

    // driver_board.update_measurements();
    update_buggy_status(motor_left.get_tick_count(), motor_right.get_tick_count());

    // pc.printf("%.4f,%.4f\n", motor_left.get_speed(), motor_left.get_filtered_speed());
    // pc.printf("%.4f\n", buggy_status.cumulative_angle_deg);

    /* Calculate and apply PID output on certain modes only*/
    if (buggy_state == PID_test ||
        buggy_state == square_mode || 
        buggy_state == line_follow)
    {
        // Update set PID_angle depending on buggy mode
        if (buggy_state == square_mode || buggy_state == PID_test)
        {
            PID_angle.update(buggy_status.set_angle, buggy_status.cumulative_angle_deg);
        }
        else if (buggy_state == line_follow)
        {
            PID_angle.update(0, sensor_array.get_angle_output());
        }

        // Calculate Motor Set Speeds
        buggy_status.left_set_speed  = buggy_status.set_velocity + PID_angle.get_output();
        buggy_status.right_set_speed = buggy_status.set_velocity - PID_angle.get_output();

        // Calculate Motor PID and apply the output: 
        PID_motor_left.update(buggy_status.left_set_speed, motor_left.get_filtered_speed());
        PID_motor_right.update(buggy_status.right_set_speed, motor_right.get_filtered_speed());
        motor_left.set_duty_cycle(PID_motor_left.get_output());
        motor_right.set_duty_cycle(PID_motor_right.get_output());

        // Sends PID Data to the PC
        float* out_arr = PID_angle.get_terms();
        pc.printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", 
                        out_arr[0], //= time_index
                        out_arr[1], //= set_point
                        out_arr[2], //= measurement
                        out_arr[3], //= error
                        out_arr[4], //= propotional
                        out_arr[5], //= integrator
                        out_arr[6], //= differentiator
                        out_arr[7]); //= output
    }

    // Measure control ISR execution time
    ISR_exec_time = global_timer.read_us() - curr_time;
}


void serial_update_ISR(void)
{
    pc_serial_update = true;
    bt_serial_update = true;
}


/* MAIN FUNCTION */
int main()
{
    while (!bt.is_ready()) {};          // while bluetooth not ready, loop and do nothing

    driver_board.disable();
    sensor_array.set_all_led_on(true);
    // pc.printf("set_point,measurement,error,proportional,integrator,differentiator,output\n");

    global_timer.start();                                                           // Starts the global program timer
    control_ticker.attach_us(&control_update_ISR, CONTROL_UPDATE_PERIOD_US);        // Starts the control ISR update ticker
    serial_ticker.attach(&serial_update_ISR, SERIAL_UPDATE_PERIOD);                 // Starts the control ISR update ticker

    while (1)
    {
        int curr_time = global_timer.read_us();

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
                                break;
                            case bt.gains_PID:
                                switch (bt.obj_type)
                                {
                                    case bt.motor_left:
                                        PID_motor_left.set_constants(bt.data1, bt.data2, bt.data3);
                                        break;
                                    case bt.motor_right:
                                        PID_motor_right.set_constants(bt.data1, bt.data2, bt.data3);
                                        break;
                                    case bt.sensor:
                                        PID_angle.set_constants(bt.data1, bt.data2, bt.data3);
                                        break;
                                    default:
                                        break;
                                }
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
                                buggy_state = inactive;
                                stop_motors();
                                break;
                            case bt.uturn:
                                buggy_state = inactive; // WIP
                                break;
                            case bt.encoder_test:
                                stop_motors();
                                bt.set_continous(true);
                                bt.data_type_sent = bt.ticks_cumulative;
                                bt.obj_type_sent = bt.motor_both;
                                buggy_state = task_test;
                                break;
                            case bt.motor_pwm_test:
                                stop_motors();
                                bt.set_continous(true);
                                bt.data_type_sent = bt.pwm_duty;
                                bt.obj_type_sent = bt.motor_both;
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
                                buggy_status.set_angle = 0;
                                buggy_status.set_velocity = 0.0;
                                buggy_state = PID_test;
                                break;
                            case bt.toggle_led_test:
                                LED = !LED;
                                buggy_state = task_test_inactive;
                                break;
                            case bt.line_follow:
                                reset_everything();
                                buggy_state = line_follow;
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
                bt.send_fstring("Err: %s", bt.get_data());
            }
            bt.reset_rx_buffer();
        }

        if (pc.readable())
        {
            switch (pc.getc()) 
            {
            case 'r':
                reset_everything();
                buggy_state = line_follow;
                break;
            case 's':
                buggy_state = inactive;
                stop_motors();
                break;
            default:
                break;
            }

        }
        /* ---  END OF BLUETOOTH COMMAND HANDLING  --- */


        /* --- START OF BUGGY ACTIONS/STATE LOGIC CODE --- */ 
        driver_board.set_enable(buggy_state != inactive && buggy_state != task_test_inactive);
        switch (buggy_state) 
        {   
            case inactive:
                stop_motors();
                driver_board.disable();
                break;
            case square_mode:
                switch (square_task.stage)
                {
                    case 0:
                        square_task.set_distance += SQUARE_DISTANCE;
                        buggy_status.set_angle = square_task.set_angle;
                        buggy_status.set_velocity = SQUARE_VELOCITY_SET;
                        square_task.stage++;
                        break;
                    case 7:
                    case 1:
                    case 3:
                    case 5:
                        if (buggy_status.distance_travelled >= square_task.set_distance) // wait to move 1m then, start turning right
                        {
                            if (square_task.stage == 7)
                            {
                                square_task.set_angle += SQUARE_TURNING_RIGHT_ANGLE;
                            }
                            square_task.set_angle += SQUARE_TURNING_RIGHT_ANGLE;
                            buggy_status.set_angle = square_task.set_angle;
                            buggy_status.set_velocity = 0;
                            square_task.stage++;
                        }
                        break;
                    case 2:
                    case 4:
                    case 6:
                    case 8:
                        if (buggy_status.cumulative_angle_deg >= square_task.set_angle) // wait to turn 90 and start moving straight
                        {
                            square_task.set_distance += SQUARE_DISTANCE;
                            buggy_status.set_angle = square_task.set_angle;
                            buggy_status.set_velocity = SQUARE_VELOCITY_SET;
                            square_task.stage++;
                        }
                        break;
                    case 9:
                    case 11:
                    case 13:
                        if (buggy_status.distance_travelled >= square_task.set_distance) // wait to move 1m then, start turning left
                        {
                            square_task.set_angle -= SQUARE_TURNING_LEFT_ANGLE;
                            buggy_status.set_angle = square_task.set_angle;
                            buggy_status.set_velocity = 0;
                            square_task.stage++;
                        }
                        break;
                    case 10:
                    case 12:
                    case 14:
                        if (buggy_status.cumulative_angle_deg <= square_task.set_angle) // wait to turn -90 and start moving straight
                        {
                            square_task.set_distance += SQUARE_DISTANCE;
                            buggy_status.set_angle = square_task.set_angle;
                            buggy_status.set_velocity = SQUARE_VELOCITY_SET;
                            square_task.stage++;
                        }
                        break;
                    case 15:
                        if (buggy_status.distance_travelled >= square_task.set_distance)  // Stop
                        {
                            buggy_state = inactive;
                            stop_motors();
                            square_task.stage = 0;
                        }
                        break;
                    default:
                        break;
                }
                break;
            case PID_test:
                if (buggy_status.distance_travelled <= 0.1)
                {
                    buggy_status.set_velocity = 0.1;
                }
                if (buggy_status.distance_travelled >= 0.1)
                {
                    buggy_status.set_velocity = 0.3;
                }
                if (buggy_status.distance_travelled >= 0.3)
                {
                    buggy_status.set_velocity = 0.5;
                }
                if (buggy_status.distance_travelled >= 0.6)
                {
                    buggy_state = inactive;
                    reset_everything();
                }
                break;
            case line_follow:
                buggy_status.set_velocity = LINE_FOLLOW_VELOCITY;
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
                switch (bt.data_type_sent)
                {
                    case bt.pwm_duty:               // P
                        switch (bt.obj_type_sent)
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
                        switch (bt.obj_type_sent)
                        {
                            case bt.motor_left:
                                bt.send_fstring("Ticks L: %d", motor_left.get_tick_count());
                                break;
                            case bt.motor_right:
                                bt.send_fstring("Ticks R: %d", motor_right.get_tick_count());
                                break;
                            case bt.motor_both:
                                bt.send_fstring("L:%7d R:%7d", motor_left.get_tick_count(), motor_right.get_tick_count());
                                break;
                            default:
                                break;
                        }
                        break;
                    case bt.speed:                  // V
                        switch (bt.obj_type_sent)
                        {
                            case bt.motor_left:
                                bt.send_fstring("Speed L: %f", motor_left.get_speed());
                                break;
                            case bt.motor_right:
                                bt.send_fstring("Speed R: %f", motor_right.get_speed());
                                break;
                            case bt.motor_both:
                                bt.send_fstring("S L:%.3f/ R:%.3f", motor_left.get_speed(), motor_right.get_speed());
                                break;
                            default:
                                break;
                        }
                        break;
                    case bt.gains_PID:              // G
                        bt.send_fstring("P:%.3f,I:%.3f,D:%.3f", bt.data1, bt.data2, bt.data3);
                        break;
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
            // pc.printf("\n");
            // pc.printf("\n");

            // pc.printf("                        L   |   R   \n");
            // pc.printf("Duty Cycle:          %6.2f | %6.2f \n",  motor_left.get_duty_cycle(), motor_right.get_duty_cycle());
            // pc.printf("Encoder Ticks:       %6d | %6d \n",      motor_left.get_tick_count(), motor_right.get_tick_count());
            // pc.printf("Motor Speed (m/s):   %6.4f | %6.4f \n",  motor_left.get_speed(), motor_right.get_speed());

            // pc.printf("\n");

            // pc.printf("Cumulative Angle:    %7.4f Degrees \n", buggy_status.cumulative_angle_deg);
            // pc.printf("Distance Travelled:  %7.4f Metres \n", buggy_status.distance_travelled);
            // pc.printf("Sensor Values:  \n");

            // pc.printf("Calculation ISR Time: %d us / Main Loop Time: %d us / Global Time: %d us \n", ISR_exec_time, loop_exec_time, global_timer.read_us());

            // float* sens = sensor_array.get_sens_output_array();
            // for (int i = 0; i < 6; i++)
            // {
            //     pc.printf("%d:%6.4f,", i, sens[i]);
            // }
            // pc.printf("Out:%f\n", sensor_array.get_array_output());

            // pc.printf("%d, %d\n", ISR_exec_time, loop_exec_time);

            pc_serial_update = false;
        }
        /* ---  END OF SERIAL UPDATE CODE  --- */


        /*       END OF LOOP      */
        loop_exec_time = global_timer.read_us() - curr_time;
    }
}
