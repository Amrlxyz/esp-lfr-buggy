#include "mbed.h"
#include "QEI.h"

#include "constants.h"
#include "pin_assignments.h"
#include "bluetooth.h"
#include "motor.h"
#include "PID.h"
#include "motor_driver_board.h"


class SensorArray
{
protected:

    DigitalOut led0_;
    DigitalOut led1_;
    DigitalOut led2_;
    DigitalOut led3_;
    DigitalOut led4_;
    DigitalOut led5_;

    AnalogIn sens0_;
    AnalogIn sens1_;
    AnalogIn sens2_;
    AnalogIn sens3_;
    AnalogIn sens4_;
    AnalogIn sens5_;

    float output;
    float angle_output;
    float sens_values[6];

public:

    SensorArray(PinName sens0, PinName sens1, PinName sens2, PinName sens3, PinName sens4, PinName sens5,
                PinName led0, PinName led1, PinName led2, PinName led3, PinName led4, PinName led5):
                sens0_(sens0), led0_(led0),
                sens1_(sens1), led1_(led1),
                sens2_(sens2), led2_(led2),
                sens3_(sens3), led3_(led3),
                sens4_(sens4), led4_(led4),
                sens5_(sens5), led5_(led5) 
                {
                    reset();
                    set_all_led_on(false);
                };

    
    void reset(void)
    {
        for (int i = 0; i < sizeof(sens_values) / sizeof(sens_values[0]); i++) 
        {
            sens_values[i] = 0;
        }
        output = 0;
        angle_output = 0;
    }


    void set_all_led_on(bool status)
    {
        led0_ = status;
        led1_ = status;
        led2_ = status;
        led3_ = status;
        led4_ = status;
        led5_ = status;
    }

    
    float read(AnalogIn sensor){return 0;}

    
    void update(void)
    {
        float val_0_sample_total = 0;
        float val_1_sample_total = 0;
        float val_2_sample_total = 0;
        float val_3_sample_total = 0;
        float val_4_sample_total = 0;
        float val_5_sample_total = 0;

        for (int i = 0; i < SENS_SAMPLE_COUNT; i++)
        {            
            val_0_sample_total += sens0_.read(); 
            val_1_sample_total += sens1_.read();
            val_2_sample_total += sens2_.read();
            val_3_sample_total += sens3_.read();
            val_4_sample_total += sens4_.read();
            val_5_sample_total += sens5_.read();
        }

        sens_values[0] = val_0_sample_total / SENS_SAMPLE_COUNT;
        sens_values[1] = val_1_sample_total / SENS_SAMPLE_COUNT;
        sens_values[2] = val_2_sample_total / SENS_SAMPLE_COUNT;
        sens_values[3] = val_3_sample_total / SENS_SAMPLE_COUNT;
        sens_values[4] = val_4_sample_total / SENS_SAMPLE_COUNT;
        sens_values[5] = val_5_sample_total / SENS_SAMPLE_COUNT;

        output = (sens_values[0] * (-3) + sens_values[1] * (-2) + sens_values[2] * (-1) + sens_values[3] * (1) + sens_values[4] * (2) + sens_values[5] * (3));

        angle_output = output * (SENS_ANGLE_COEFF);
    }

    
    float get_sens_output(int index)
    {
        if ((index < sizeof(sens_values) / sizeof(sens_values[0])) && (index >= 0))
        {
            return sens_values[index]; 
        }
        return -1;
    }

    float* get_sens_output_array(void)
    {
        return sens_values;
    } 

    float get_array_output(void)
    {
        return output;
    }

    float get_angle_output(void)
    {
        return angle_output;
    }
};


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

Bluetooth bt(BT_TX_PIN, BT_RX_PIN);     
MotorDriverBoard driver_board(DRIVER_ENABLE_PIN, DRIVER_MONITOR_PIN);
SensorArray sensor_array(SENSOR0_IN_PIN, SENSOR1_IN_PIN, SENSOR2_IN_PIN, SENSOR3_IN_PIN, SENSOR4_IN_PIN, SENSOR5_IN_PIN,
                        SENSOR0_OUT_PIN, SENSOR1_OUT_PIN, SENSOR2_OUT_PIN, SENSOR3_OUT_PIN, SENSOR4_OUT_PIN, SENSOR5_OUT_PIN);
Motor motor_left(MOTORL_PWM_PIN, MOTORL_DIRECTION_PIN, MOTORL_BIPOLAR_PIN, MOTORL_CHA_PIN, MOTORL_CHB_PIN);
Motor motor_right(MOTORR_PWM_PIN , MOTORR_DIRECTION_PIN, MOTORR_BIPOLAR_PIN, MOTORR_CHA_PIN, MOTORR_CHB_PIN);
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
    driver_board.update_measurements();
    update_buggy_status(motor_left.get_tick_count(), motor_right.get_tick_count());

    // pc.printf("%.4f,%.4f\n", motor_left.get_speed(), motor_left.get_filtered_speed());
    // pc.printf("%.4f\n", buggy_status.cumulative_angle_deg);

    if (buggy_state == square_mode)
    {
        PID_angle.update(buggy_status.set_angle, buggy_status.cumulative_angle_deg);
    }
    else if (buggy_state == line_follow)
    {
        PID_angle.update(0, sensor_array.get_angle_output());
    }

    /* Calculate and apply PID output on certain modes only*/
    if (buggy_state == PID_test || buggy_state == square_mode)
    {
        buggy_status.left_set_speed  = buggy_status.set_velocity + PID_angle.get_output();
        buggy_status.right_set_speed = buggy_status.set_velocity - PID_angle.get_output();    

        /* PID Calculations: */
        PID_motor_left.update(buggy_status.left_set_speed, motor_left.get_filtered_speed());
        PID_motor_right.update(buggy_status.left_set_speed, motor_right.get_filtered_speed());

        motor_left.set_duty_cycle(PID_motor_left.get_output());
        motor_right.set_duty_cycle(PID_motor_right.get_output());

        // pc.printf("SetL:%.4f,SetR:%.4f,FiltL:%.4f,ErrL:%.4f,OutL:%.4f,OutR:%.4f\n", buggy_status.left_set_speed, buggy_status.left_set_speed, motor_left.get_filtered_speed(), buggy_status.left_set_speed - motor_left.get_filtered_speed(), PID_motor_left.get_output(), PID_motor_right.get_output());
        // pc.printf("R - Meas: %f, Out: %f\n", motor_right.get_speed(), PID_motor_right.get_output());
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
                                buggy_status.set_angle = 90;
                                buggy_status.set_velocity = 0.0;
                                buggy_state = PID_test;
                                break;
                            case bt.toggle_led_test:
                                LED = !LED;
                                buggy_state = task_test_inactive;
                                break;
                            case bt.line_follow:
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
                bt.send_fstring("Invalid Command", bt.get_data());
            }
            bt.reset_rx_buffer();
        }
        /* ---  END OF BLUETOOTH COMMAND HANDLING  --- */


        /* --- START OF BUGGY ACTIONS/STATE LOGIC CODE --- */ 
        driver_board.update_enable(buggy_state != inactive && buggy_state != task_test_inactive);
        switch (buggy_state) 
        {   
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
                if (buggy_status.distance_travelled >= 0.5)
                {
                    buggy_state = inactive;
                    reset_everything();
                }
                break;
            case line_follow:
                sensor_array.get_angle_output();
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

            float* sens = sensor_array.get_sens_output_array();
            for (int i = 0; i < 6; i++)
            {
                pc.printf("%d:%6.4f,", i, sens[i]);
            }
            pc.printf("Out:%f\n", sensor_array.get_array_output());

            pc_serial_update = false;
        }
        /* ---  END OF SERIAL UPDATE CODE  --- */


        /*       END OF LOOP      */
        loop_exec_time = global_timer.read_us() - curr_time;
    }
}
