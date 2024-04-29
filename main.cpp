/**
 * @file main.cpp
 * @brief main buggy code
 *
 * This file contains all the buggy logic
 * 
 */

#include "mbed.h"
#include "QEI.h"

#include "constants.h"
#include "pin_assignments.h"
#include "bluetooth.h"
#include "motor.h"
#include "PID.h"
#include "motor_driver_board.h"
#include "sensor_array.h"


/* BT COMMAND CHARS */
enum Bt_cmd_chars
{
    // 1 - cmd types
    ch_execute = 'E',                // E
    ch_get = 'G',                    // G
    ch_set = 'S',                    // S
    ch_continous = 'C',              // C

    // 2 - exec types
    ch_stop = 'S',                   // S
    ch_active_stop = 'X',            // X
    ch_uturn = 'U',                  // U
    ch_encoder_test = 'E',           // E
    ch_motor_pwm_test = 'M',         // M
    ch_straight_test = 'Z',          // C
    ch_square_test = 'Q',            // Q
    ch_PID_test = 'P',               // P
    ch_toggle_led_test = 'L',        // L
    ch_line_follow = 'F',            // F
    ch_static_tracking = 'T',        // T
    ch_line_follow_auto = 'A',       // A
    ch_calibrate = 'C',

    // 2 - data types
    ch_pwm_duty = 'D',               // D
    ch_ticks_cumulative = 'E',       // E
    ch_speed = 'S',                  // S
    ch_gains_PID = 'P',              // P
    ch_tau_PID = 'T',                // T
    ch_current_usage = 'C',          // C
    ch_runtime = 'R',                // R
    ch_loop_time = 'X',              // X
    ch_loop_count = 'Y',             // Y

    // 3 - obj types
    ch_motor_left = 'L',         // L // PID, Encoder Ticks, Velocity
    ch_motor_right = 'R',        // R // PID, Encoder Ticks, Velocity
    ch_motor_both = 'B',         // B
    ch_sensor = 'S',             // S
    ch_no_obj = 'D',             // default case
};


/* BUGGY MODES */
enum Buggy_modes
{
    square_mode,
    straight_test,
    PID_test,
    line_test,
    line_follow,
    task_test,
    task_test_inactive,
    inactive,
    active_stop,
    uturn,
    static_tracking,
    line_follow_auto,
    stop_detect_line,
    calibration,
};


/**
 * @brief Represents the status of a buggy.
 * 
 * This struct holds various parameters related to the state of the buggy, including velocities,
 * angles, distances, and task-specific variables.
 */
struct Buggy_status
{
    float set_velocity;         /**< @brief The set velocity for the buggy. */
    float set_angle;            /**< @brief The set angle for the buggy. */

    float left_set_speed;       /**< @brief The set speed of the left wheel. */
    float right_set_speed;      /**< @brief The set speed of the right wheel. */

    float cumulative_angle_deg; /**< @brief The cumulative angle (in degrees) traveled by the buggy. */
    float distance_travelled;   /**< @brief The distance travelled by the buggy. */

    float lf_line_last_seen;    /**< @brief The position of the last seen line by the left front sensor. */

    // Accel Curve Variables
    float accel_start_distance;
    float accel_start_angle;
    bool is_accelerating;

    // square task variables
    float sq_set_angle;         /**< @brief The set angle for the square task. */
    float sq_set_distance;      /**< @brief The set distance for the square task. */
    int sq_stage;               /**< @brief The current stage of the square task. */
};


/* GLOBAL VARIBLES DECLARATIONS */
volatile bool pc_serial_update = false;
volatile bool bt_serial_update = false;
int ISR_exec_time = 0;
int loop_exec_time = 0;

float bt_float_data[3] = {0};
short int data_log[LOG_SIZE][5] = {0};
int log_index = 0;
char bt_data_sent;
char bt_obj_sent;
float* pid_constants;               // used only for sending datat to bluetooth
float* cal_constants;

Buggy_modes  buggy_mode;          // stores buggy states when performing actions
Buggy_modes  prev_buggy_mode;
Buggy_status buggy_status = {0};

volatile float lf_velocity = LINE_FOLLOW_VELOCITY;


/* OBJECTS DECLARATIONS */
DigitalOut LED(LED_PIN);                    // Debug LED set
Serial pc(USBTX, USBRX, 115200);            // set up serial comm with pc
Timer global_timer;                         // set up global program timer
Ticker control_ticker;
Ticker serial_ticker;
Ticker sensor_ticker;
Timeout logic_timout;

Bluetooth bt(BT_TX_PIN, BT_RX_PIN, BT_BAUD_RATE);     
MotorDriverBoard driver_board(DRIVER_ENABLE_PIN, DRIVER_MONITOR_PIN);
SensorArray sensor_array(SENSOR0_IN_PIN, SENSOR1_IN_PIN, SENSOR2_IN_PIN, SENSOR3_IN_PIN, SENSOR4_IN_PIN, SENSOR5_IN_PIN,
                         SENSOR0_OUT_PIN, SENSOR1_OUT_PIN, SENSOR2_OUT_PIN, SENSOR3_OUT_PIN, SENSOR4_OUT_PIN, SENSOR5_OUT_PIN, SENS_SAMPLE_COUNT, SENS_DETECT_RANGE, SENS_ANGLE_COEFF);
Motor motor_left (MOTORL_PWM_PIN, MOTORL_DIRECTION_PIN, MOTORL_BIPOLAR_PIN, MOTORL_CHA_PIN, MOTORL_CHB_PIN, PULSE_PER_REV, MOTOR_PWM_FREQ, CONTROL_UPDATE_RATE, LP_SPEED_A0, LP_SPEED_B0, LP_SPEED_B1, WHEEL_RADIUS);
Motor motor_right(MOTORR_PWM_PIN, MOTORR_DIRECTION_PIN, MOTORR_BIPOLAR_PIN, MOTORR_CHA_PIN, MOTORR_CHB_PIN, PULSE_PER_REV, MOTOR_PWM_FREQ, CONTROL_UPDATE_RATE, LP_SPEED_A0, LP_SPEED_B0, LP_SPEED_B1, WHEEL_RADIUS);
PID PID_motor_left (PID_M_L_KP, PID_M_L_KI, PID_M_L_KD, PID_M_TAU, PID_M_MIN_OUT, PID_M_MAX_OUT, PID_M_MIN_INT, PID_M_MAX_INT, CONTROL_UPDATE_PERIOD);
PID PID_motor_right(PID_M_R_KP, PID_M_R_KI, PID_M_R_KD, PID_M_TAU, PID_M_MIN_OUT, PID_M_MAX_OUT, PID_M_MIN_INT, PID_M_MAX_INT, CONTROL_UPDATE_PERIOD);
PID PID_angle (PID_A_KP, PID_A_KI, PID_A_KD, PID_A_TAU, PID_A_MIN_OUT, PID_A_MAX_OUT, PID_A_MIN_INT, PID_A_MAX_INT, CONTROL_UPDATE_PERIOD);
PID PID_sensor(PID_S_KP, PID_S_KI, PID_S_KD, PID_S_TAU, PID_S_MIN_OUT, PID_S_MAX_OUT, PID_S_MIN_INT, PID_S_MAX_INT, SENSOR_UPDATE_PERIOD);


// Helper Function Prototypes:
void stop_motors(void);                                                 ///< Set pwm dc to 0 for both motors
void update_buggy_status(int tick_count_left, int tick_count_right);    ///< Update buggy status variables
void reset_everything(void);                                            ///< Reset all buggy values and all objects variables
bool bt_parse_rx(char* rx_buffer);                                      ///< Parse recieved bluetooth data
void control_update_ISR(void);                                          ///< ISR updating the control algorithm
void serial_update_ISR(void);                                           ///< ISR to update flag to send data to pc/bt in main()
void stop_detect_ISR(void);                                     
void bt_send_data(void);                                                ///< Send data to the bt module
void pc_send_data(void);                                                ///< Send data to the pc
void sensor_update_ISR();
void slow_accel_ISR(void);


/* MAIN FUNCTION */
int main()
{
    buggy_mode = inactive;
    buggy_mode = inactive;

    while (!bt.is_ready()) {};          // while bluetooth not ready, loop and do nothing

    sensor_array.set_all_led_on(true);

    global_timer.start();                                                           // Starts the global program timer
    sensor_ticker.attach_us(&sensor_update_ISR, SENSOR_UPDATE_PERIOD_US);           // Starts the control ISR update ticker
    control_ticker.attach_us(&control_update_ISR, CONTROL_UPDATE_PERIOD_US);        // Starts the control ISR update ticker
    serial_ticker.attach(&serial_update_ISR, SERIAL_UPDATE_PERIOD);                 // Starts the control ISR update ticker
    
    while (1)
    {
        int curr_time = global_timer.read_us();

        /* --- START OF COMMAND PROCESSING --- */
        if (bt.data_recieved_complete()) 
        {
            char* rx_buf = bt.get_rx_buffer(); 
            if (!bt_parse_rx(rx_buf))
            {
                bt.send_fstring("Err: %s", rx_buf);
            }
            bt.reset_rx_buffer();
        }

        if (pc.readable())
        {
            switch (pc.getc()) 
            {
            case 'r':
                reset_everything();
                buggy_mode = static_tracking;
                break;
            case 's':
                buggy_mode = inactive;
                stop_motors();
                break;
            case 'D':
                buggy_mode = inactive;
                for(int i = 0; i < log_index; i++)
                {
                    pc.printf("%.5f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                                    (i * CONTROL_UPDATE_PERIOD),
                                    (float) (data_log[i][0] / 1000.0), //= set_point
                                    (float) (data_log[i][1] / 1000.0), //= measurement
                                    (float) (data_log[i][2] / 1000.0), //= propotional
                                    (float) (data_log[i][3] / 1000.0), //= integrator
                                    (float) (data_log[i][4] / 1000.0)); //= differentiator
                }
                log_index = 0;
                break;
            default:
                break;
            }

        }
        /* ---  END OF COMMAND PROCESSING  --- */


        /* --- START OF BUGGY ACTIONS/STATE LOGIC CODE --- */ 
        update_buggy_status(motor_left.get_tick_count(), motor_right.get_tick_count());

        // Buggy mode transition code
        if (buggy_mode != prev_buggy_mode)
        {
            driver_board.set_enable(buggy_mode != inactive && 
                                    buggy_mode != task_test_inactive);

            switch (buggy_mode)
            {
                case PID_test:
                    reset_everything();
                    pid_constants = PID_motor_left.get_constants();
                    bt.send_fstring("P:%.2f,I:%.2f", pid_constants[0], pid_constants[1]);
                    bt.send_fstring("D:%.2f,T:%.2f\n", pid_constants[2], pid_constants[3]);
                    buggy_status.set_angle = 0;
                    buggy_status.set_velocity = 0.0;
                    break;
                case square_mode:
                case straight_test:
                    reset_everything();
                    break;
                case task_test:
                case task_test_inactive:
                    bt.set_continous(true);
                    break;
                case uturn:
                    reset_everything();
                    buggy_status.set_angle = UTURN_ANGLE;
                    buggy_status.set_velocity = 0.0;
                    break;
                case static_tracking:
                    reset_everything();
                    pid_constants = PID_sensor.get_constants();
                    bt.send_fstring("\nP:%.3f\nI:%.3f\n", pid_constants[0], pid_constants[1]);
                    bt.send_fstring("D:%.3f\nT:%.3f\n", pid_constants[2], pid_constants[3]);
                    buggy_status.set_angle = 0;
                    buggy_status.set_velocity = 0.0;
                    break;
                case line_follow_auto:
                case line_follow:
                    reset_everything();

                    if (!sensor_array.is_line_detected())
                    {
                        buggy_mode = inactive;
                        bt.send_fstring("Line undetected\n", global_timer.read());
                        break;
                    }
                    
                    pid_constants = PID_sensor.get_constants();
                    bt.send_fstring("T:%.3f\n", global_timer.read());
                    bt.send_fstring("\nP:%.3f\nI:%.3f\n", pid_constants[0], pid_constants[1]);
                    bt.send_fstring("D:%.3f\nT:%.3f\n", pid_constants[2], pid_constants[3]);

                    buggy_status.set_angle = 0;
                    buggy_status.set_velocity = lf_velocity / SLOW_ACCEL_DIVIDER;

                    buggy_status.accel_start_angle = buggy_status.cumulative_angle_deg;
                    buggy_status.accel_start_distance = buggy_status.distance_travelled;
                    buggy_status.is_accelerating = false;
                    logic_timout.attach(&slow_accel_ISR, SLOW_ACCEL_TIME);
                    break;
                case active_stop:
                    reset_everything();
                    buggy_status.set_angle = 0;
                    buggy_status.set_velocity = 0.0;
                    bt.send_fstring("T:%.3f\n", global_timer.read());
                    break;
                case stop_detect_line:
                    reset_everything();
                    logic_timout.attach(&stop_detect_ISR, STOP_DETECT_TIME);
                    buggy_status.set_angle = 0;
                    buggy_status.set_velocity = 0.0;
                    break;
                case calibration:
                    reset_everything();
                    sensor_array.calibrate_sensors();
                    cal_constants = sensor_array.get_calibration_constants();
                    for (int i = 0; i < 6; i++)
                    {
                        bt.send_fstring("%d:%.4f\n", i, cal_constants[i]);
                    }
                    buggy_mode = inactive;
                    break;
                default:
                    break;
            }            
            prev_buggy_mode = buggy_mode;
        }

        // Buggy mode continous logic
        switch (buggy_mode) 
        {   
            case inactive:
                driver_board.disable();
                stop_motors();
                break;
            case square_mode:
                switch (buggy_status.sq_stage)
                {
                    case 0:
                        buggy_status.sq_set_distance += SQUARE_DISTANCE;
                        buggy_status.set_angle = buggy_status.sq_set_angle;
                        buggy_status.set_velocity = SQUARE_VELOCITY_SET;
                        buggy_status.sq_stage++;
                        break;
                    case 7:
                    case 1:
                    case 3:
                    case 5:
                        if (buggy_status.distance_travelled >= buggy_status.sq_set_distance) // wait to move 1m then, start turning right
                        {
                            if (buggy_status.sq_stage == 7)
                            {
                                buggy_status.sq_set_angle += SQUARE_TURNING_RIGHT_ANGLE;
                            }
                            buggy_status.sq_set_angle += SQUARE_TURNING_RIGHT_ANGLE;
                            buggy_status.set_angle = buggy_status.sq_set_angle;
                            buggy_status.set_velocity = 0;
                            buggy_status.sq_stage++;
                        }
                        break;
                    case 2:
                    case 4:
                    case 6:
                    case 8:
                        if (buggy_status.cumulative_angle_deg >= buggy_status.sq_set_angle) // wait to turn 90 and start moving straight
                        {
                            buggy_status.sq_set_distance += SQUARE_DISTANCE;
                            buggy_status.set_angle = buggy_status.sq_set_angle;
                            buggy_status.set_velocity = SQUARE_VELOCITY_SET;
                            buggy_status.sq_stage++;
                        }
                        break;
                    case 9:
                    case 11:
                    case 13:
                        if (buggy_status.distance_travelled >= buggy_status.sq_set_distance) // wait to move 1m then, start turning left
                        {
                            buggy_status.sq_set_angle -= SQUARE_TURNING_LEFT_ANGLE;
                            buggy_status.set_angle = buggy_status.sq_set_angle;
                            buggy_status.set_velocity = 0;
                            buggy_status.sq_stage++;
                        }
                        break;
                    case 10:
                    case 12:
                    case 14:
                        if (buggy_status.cumulative_angle_deg <= buggy_status.sq_set_angle) // wait to turn -90 and start moving straight
                        {
                            buggy_status.sq_set_distance += SQUARE_DISTANCE;
                            buggy_status.set_angle = buggy_status.sq_set_angle;
                            buggy_status.set_velocity = SQUARE_VELOCITY_SET;
                            buggy_status.sq_stage++;
                        }
                        break;
                    case 15:
                        if (buggy_status.distance_travelled >= buggy_status.sq_set_distance)  // Stop
                        {
                            buggy_mode = inactive;
                            stop_motors();
                            buggy_status.sq_stage = 0;
                        }
                        break;
                    default:
                        break;
                }
                break;
            case PID_test:
                if (buggy_status.distance_travelled <= 0.1)
                {
                    buggy_status.set_velocity = 2;
                }
                // if (buggy_status.distance_travelled >= 0.1)
                // {
                //     buggy_status.set_velocity = 0.5;
                // }
                // if (buggy_status.distance_travelled >= 0.4)
                // {
                //     buggy_status.set_velocity = 1;
                // }
                // if (buggy_status.distance_travelled >= 1)
                // {
                //     buggy_status.set_velocity = 0.5;
                // }
                if (buggy_status.distance_travelled >= 0.7)
                {
                    buggy_mode = active_stop;
                    reset_everything();
                }
                break;

            case uturn:
                if (buggy_status.cumulative_angle_deg >= buggy_status.set_angle)
                {
                    buggy_mode = inactive;
                    while(true)
                    {
                        if (sensor_array.is_line_detected())
                        {
                            break;
                        }
                    }
                    buggy_mode = line_follow_auto;
                }
                break;
            case line_follow_auto:
                if (sensor_array.is_line_detected())
                {
                    buggy_status.lf_line_last_seen = buggy_status.distance_travelled;
                }
                else if (buggy_status.distance_travelled - buggy_status.lf_line_last_seen >= LINE_FOLLOW_STOP_DISTANCE)
                {
                    buggy_mode = stop_detect_line;
                    stop_motors();
                }
            case line_follow:    
                //// comment this disable accel
                // if (buggy_status.is_accelerating)
                // {
                //     float accel_distance = buggy_status.distance_travelled - buggy_status.accel_start_distance;
                //     if (accel_distance > MANUAL_ACCEL_DISTANCE)
                //     {
                //         // pc.printf("SLOWING DOWN\n");
                //         buggy_status.set_velocity = lf_velocity;
                //         buggy_status.accel_start_angle = buggy_status.cumulative_angle_deg;
                //         buggy_status.is_accelerating = false;
                //     }
                //     else
                //     {
                //         // pc.printf("fast\n");
                //         buggy_status.set_velocity = MANUAL_ACCEL_SPEED;
                //     }
                // }
                // else 
                // {
                //     float accel_angle = fabsf(buggy_status.cumulative_angle_deg - buggy_status.accel_start_angle);
                //     // pc.printf("%f\n", accel_angle);
                //     if (accel_angle > MANUAL_ACCEL_ANGLE)
                //     {
                //         // pc.printf("Accelerating!!! %f\n", accel_angle);
                //         buggy_status.set_velocity = MANUAL_ACCEL_SPEED;
                //         buggy_status.accel_start_distance = buggy_status.distance_travelled;
                //         buggy_status.is_accelerating = true;
                //     }
                //     else
                //     {
                //         // pc.printf("slowstuff\n");
                //         buggy_status.set_velocity = lf_velocity;
                //     }
                // }
            case stop_detect_line:
                if (sensor_array.is_line_detected())
                {
                    buggy_mode = line_follow_auto;
                };
            default:
                break;
        }    
        /* ---  END OF BUGGY ACTIONS/STATE LOGIC CODE  --- */ 

        
        /* --- START OF SERIAL UPDATE CODE --- */
        if (bt_serial_update)
        {
            bt_send_data();
            bt_serial_update = false;
        }

        if (pc_serial_update)
        {
            pc_send_data();
            pc_serial_update = false;
        }
        /* ---  END OF SERIAL UPDATE CODE  --- */


        /*       END OF LOOP      */
        loop_exec_time = global_timer.read_us() - curr_time;
    }
}


/* HELPER FUNCTIONS */
void control_update_ISR(void)
{   
    // Get the current time to measure the execution time
    int curr_time = global_timer.read_us();

    /* Run all the update functions: */
    motor_left.update();
    motor_right.update();

    /* Calculate and apply PID output on certain modes only*/
    if (buggy_mode == PID_test ||
        buggy_mode == square_mode || 
        buggy_mode == line_follow ||
        buggy_mode == line_follow_auto ||
        buggy_mode == uturn ||
        buggy_mode == static_tracking ||
        buggy_mode == active_stop)
    {
        // Update set PID_angle and calculate motor set speed depending on buggy mode
        if (buggy_mode == square_mode || 
            buggy_mode == PID_test ||
            buggy_mode == uturn ||
            buggy_mode == active_stop)
        {
            PID_angle.update(buggy_status.set_angle, buggy_status.cumulative_angle_deg);
            
            buggy_status.left_set_speed  = buggy_status.set_velocity + PID_angle.get_output();
            buggy_status.right_set_speed = buggy_status.set_velocity - PID_angle.get_output();
        }
        else if (buggy_mode == static_tracking)
        {
            PID_sensor.update(buggy_status.set_angle, sensor_array.get_filtered_output());

            buggy_status.left_set_speed  = buggy_status.set_velocity + PID_sensor.get_output();
            buggy_status.right_set_speed = buggy_status.set_velocity - PID_sensor.get_output();
        }
        else
        {
            PID_sensor.update(buggy_status.set_angle, sensor_array.get_filtered_output());
            
            float base_speed = buggy_status.set_velocity;

            // float sens_out_abs = fabsf(sensor_array.get_filtered_output());
            // if (sens_out_abs > SLOW_TURNING_THRESH)
            // {
            //     base_speed = buggy_status.set_velocity * SLOW_TURNING_GAIN; // (1 - pid_out_abs / (PID_S_MAX_OUT * SLOW_TURNING_GAIN));
            // }
            // else
            // {
            //     base_speed = buggy_status.set_velocity;
            // }

            // buggy_status.left_set_speed  = base_speed + PID_sensor.get_output();
            // buggy_status.right_set_speed = base_speed - PID_sensor.get_output();


            float speed_offset = PID_sensor.get_output();
            if (speed_offset > 0)
            {
                buggy_status.left_set_speed  = base_speed;
                buggy_status.right_set_speed = base_speed - 2 * speed_offset;
            }
            else 
            {
                buggy_status.left_set_speed  = base_speed - 2 * -speed_offset;
                buggy_status.right_set_speed = base_speed;
            }
        }

        // Calculate Motor PID and apply the output: 
        PID_motor_left.update(buggy_status.left_set_speed, motor_left.get_filtered_speed());
        PID_motor_right.update(buggy_status.right_set_speed, motor_right.get_filtered_speed());
        motor_left.set_duty_cycle(PID_motor_left.get_output());
        motor_right.set_duty_cycle(PID_motor_right.get_output());

        // PID Data Logging
        if(log_index < LOG_SIZE)
        {
            float** out_arr = PID_motor_left.get_terms();
            // float** out_arr = PID_motor_right.get_terms();
            // float** out_arr = PID_sensor.get_terms();
            // data_log[log_index][0] = *out_arr[0]; //= time_index
            data_log[log_index][0] = (short int) (*out_arr[1] * 1000); //= set_point
            data_log[log_index][1] = (short int) (*out_arr[2] * 1000); //= measurement
            // data_log[log_index][3] = *out_arr[3]; //= error
            data_log[log_index][2] = (short int) (*out_arr[4] * 1000); //= propotional
            data_log[log_index][3] = (short int) (*out_arr[5] * 1000); //= integrator
            data_log[log_index][4] = (short int) (*out_arr[6] * 1000); //= differentiator
            // data_log[log_index][7] = *out_arr[7]; //= output
            log_index++;
        }

        // Sends PID Data to the PC (WARNING: CAN CAUSE BT MALFUNCTION)
        // float** out_arr = PID_motor_left.get_terms();
        // pc.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", 
        //                 *out_arr[0], //= time_index
        //                 *out_arr[1], //= set_point
        //                 *out_arr[2], //= measurement
        //                 *out_arr[3], //= error
        //                 *out_arr[4], //= propotional
        //                 *out_arr[5], //= integrator
        //                 *out_arr[6], //= differentiator
        //                 *out_arr[7]); //= output

        // float** out_arr = PID_sensor.get_terms();
        // pc.printf("o:%.2f,", *out_arr[7]);
        // pc.printf("d:%.2f\n", *out_arr[6]);

        // pc.printf("o:%.2f,", sensor_array.get_array_output());
        // pc.printf("f:%.2f\n", sensor_array.get_filtered_output());

    }

    // Motor LP Filter Debug:
    // pc.printf("%.4f,%.4f\n", motor_left.get_speed(), motor_left.get_filtered_speed());
    // pc.printf("%.4f\n", buggy_status.cumulative_angle_deg);

    // Measure control ISR execution time
    ISR_exec_time = global_timer.read_us() - curr_time;
}


void sensor_update_ISR(void)
{
    sensor_array.update();
}


void slow_accel_ISR(void)
{
    if (buggy_mode == line_follow_auto ||
        buggy_mode == line_follow)
    {
        buggy_status.set_velocity = lf_velocity;
    }
}

void stop_detect_ISR(void)
{
    if (buggy_mode == stop_detect_line)
    {
        buggy_mode = inactive;
    }
}


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
    PID_angle.reset();
    PID_sensor.reset();

    memset(&buggy_status, 0, sizeof(buggy_status));
}


bool bt_parse_rx(char* rx_buffer)
{   
    /*  This function reads the incoming data and checks for 
        specific command format and returns the command type 
        returns false if parsing failed     */

    // command parsing -> to-do 

    char cmd_type = rx_buffer[0];
    char exec_type = rx_buffer[1];
    char data_type = rx_buffer[1];
    char obj_type = rx_buffer[2];

    int data_amount;

    switch(rx_buffer[0])
    {
        case ch_continous:
            bt.set_continous(!bt.is_continous());
            break;
        case ch_get:
            bt.set_send_once(true);
            bt_data_sent = data_type;
            bt_obj_sent = obj_type;
            break;
        case ch_set:  
            data_amount = (data_type == ch_gains_PID) ? 3 : 1;
            if (sscanf(rx_buffer, "%*s %f %f %f", &bt_float_data[0], &bt_float_data[1], &bt_float_data[2]) != data_amount)
            {
                return false;
            }

            switch (data_type)
            {
                case ch_pwm_duty:               // P
                    switch (obj_type)
                    {
                        case ch_motor_left:
                            motor_left.set_duty_cycle(bt_float_data[0]);
                            break;
                        case ch_motor_right:
                            motor_right.set_duty_cycle(bt_float_data[0]);
                            break;
                        default:
                            break;
                    }
                    break;
                case ch_speed:                  // S
                    lf_velocity = bt_float_data[0];
                    break;
                case ch_gains_PID:
                    switch (obj_type)
                    {
                        case ch_motor_left:
                            PID_motor_left.set_constants(bt_float_data[0], bt_float_data[1], bt_float_data[2]);
                            break;
                        case ch_motor_right:
                            PID_motor_right.set_constants(bt_float_data[0], bt_float_data[1], bt_float_data[2]);
                            break;
                        case ch_motor_both:
                            PID_motor_left.set_constants(bt_float_data[0], bt_float_data[1], bt_float_data[2]);
                            PID_motor_right.set_constants(bt_float_data[0], bt_float_data[1], bt_float_data[2]);
                            break;
                        case ch_sensor:
                            PID_sensor.set_constants(bt_float_data[0], bt_float_data[1], bt_float_data[2]);
                            break;
                        default:
                            break;
                    }
                    break;
                case ch_tau_PID:
                    switch (obj_type)
                    {
                        case ch_motor_left:
                            PID_motor_left.set_tau(bt_float_data[0]);
                            break;
                        case ch_motor_right:
                            PID_motor_right.set_tau(bt_float_data[0]);
                            break;
                        case ch_motor_both:
                            PID_motor_left.set_tau(bt_float_data[0]);
                            PID_motor_right.set_tau(bt_float_data[0]);
                            break;
                        case ch_sensor:
                            PID_sensor.set_tau(bt_float_data[0]);
                            break;
                        default:
                            break;
                    }
                    break;
                default:
                    break;
            }
            break;
        case ch_execute:
            switch (exec_type)
            {
                case ch_stop:
                    buggy_mode = inactive;
                    break;
                case ch_active_stop:
                    buggy_mode = active_stop;
                    break;
                case ch_uturn:
                    buggy_mode = uturn;
                    break;
                case ch_encoder_test:
                    bt_data_sent = ch_ticks_cumulative;
                    bt_obj_sent = ch_motor_both;
                    buggy_mode = task_test;
                    break;
                case ch_motor_pwm_test:
                    bt_data_sent = ch_pwm_duty;
                    bt_obj_sent = ch_motor_both;
                    buggy_mode = task_test_inactive;
                    break;
                case ch_straight_test:
                    buggy_mode = straight_test;
                    break;
                case ch_square_test:
                    buggy_mode = square_mode;
                    break;
                case ch_PID_test:
                    buggy_mode = PID_test;
                    break;
                case ch_toggle_led_test:
                    LED = !LED;
                    break;
                case ch_line_follow:
                    buggy_mode = line_follow;
                    break;
                case ch_static_tracking:
                    buggy_mode = static_tracking;
                    break;
                case ch_line_follow_auto:
                    buggy_mode = line_follow_auto;
                    break;
                case ch_calibrate:
                    buggy_mode = calibration;
                    break;
                default:
                    break;
            }
            break;

        default:
            return false;
            break;
    }
    return true;
}


void serial_update_ISR(void)
{
    pc_serial_update = true;
    bt_serial_update = true;
}


void bt_send_data(void)
{
    // Handling sending data through BT 
    if (bt.is_continous() || bt.is_send_once())
    {   
        switch (bt_data_sent)
        {
            case ch_pwm_duty:               // P
                switch (bt_obj_sent)
                {
                    case ch_motor_left:
                        bt.send_fstring("DC L: %.2f", motor_left.get_duty_cycle());
                        break;
                    case ch_motor_right:
                        bt.send_fstring("DC R: %.2f", motor_right.get_duty_cycle());
                        break;
                    case ch_motor_both:
                        bt.send_fstring("DC L:%.2f/ R:%.2f", motor_left.get_duty_cycle(), motor_right.get_duty_cycle());
                        break;
                    default:
                        break;
                }
                break;
            case ch_ticks_cumulative:       // T
                switch (bt_obj_sent)
                {
                    case ch_motor_left:
                        bt.send_fstring("Ticks L: %d", motor_left.get_tick_count());
                        break;
                    case ch_motor_right:
                        bt.send_fstring("Ticks R: %d", motor_right.get_tick_count());
                        break;
                    case ch_motor_both:
                        bt.send_fstring("L:%7d R:%7d", motor_left.get_tick_count(), motor_right.get_tick_count());
                        break;
                    default:
                        break;
                }
                break;
            case ch_speed:                  // V
                switch (bt_obj_sent)
                {
                    case ch_motor_left:
                        bt.send_fstring("Speed L: %f", motor_left.get_speed());
                        break;
                    case ch_motor_right:
                        bt.send_fstring("Speed R: %f", motor_right.get_speed());
                        break;
                    case ch_motor_both:
                        bt.send_fstring("S L:%.3f/ R:%.3f", motor_left.get_speed(), motor_right.get_speed());
                        break;
                    default:
                        break;
                }
                break;
            case ch_gains_PID:              // G
                switch (bt_obj_sent)
                {
                    case ch_motor_left:
                        pid_constants = PID_motor_left.get_constants();
                        break;
                    case ch_motor_right:
                        pid_constants = PID_motor_right.get_constants();
                        break;
                    case ch_sensor:
                        pid_constants = PID_sensor.get_constants();
                        break;
                    default:
                        break;
                }
                bt.send_fstring("P:%.2f,I:%.2f", pid_constants[0], pid_constants[1]);
                bt.send_fstring("D:%.2f,T:%.2f\n", pid_constants[2], pid_constants[3]);
                break;
            case ch_current_usage:          // C          
                driver_board.update_measurements();
                bt.send_fstring("%.3fV, %.3fA\n", driver_board.get_voltage(), driver_board.get_current());
                break;
            case ch_runtime:                // R
                bt.send_fstring("Runtime: %f", global_timer.read());
                break;
            case ch_loop_time:              // X
                bt.send_fstring("ISR: %dus", ISR_exec_time);
                break;
            case ch_loop_count:             // Y
                bt.send_fstring("removed feature");
                break;
            default:
                bt.send_fstring("Err: No Data\n");
                break;
        }
        bt.set_send_once(false); 
    }
}


void pc_send_data(void)
{
    // pc.printf("\n");

    //// ---- General Troubleshoot Data ---- ////

    // pc.printf("                        L   |   R   \n");
    // pc.printf("Duty Cycle:          %6.2f | %6.2f \n",  motor_left.get_duty_cycle(), motor_right.get_duty_cycle());
    // pc.printf("Encoder Ticks:       %6d | %6d \n",      motor_left.get_tick_count(), motor_right.get_tick_count());
    // pc.printf("Motor Speed (m/s):   %6.4f | %6.4f \n",  motor_left.get_speed(), motor_right.get_speed());

    // pc.printf("\n");

    // pc.printf("Cumulative Angle:    %7.4f Degrees \n", buggy_status.cumulative_angle_deg);
    // pc.printf("Distance Travelled:  %7.4f Metres \n", buggy_status.distance_travelled);
    // pc.printf("Sensor Values:  \n");

    // pc.printf("Calculation ISR Time: %d us / Main Loop Time: %d us / Global Time: %d us \n", ISR_exec_time, loop_exec_time, global_timer.read_us());
    
    // pc.printf("%d, %d\n", ISR_exec_time, loop_exec_time);


    //// ---- Sensor Output Data ---- ////

    // float* sens = sensor_array.get_sens_output_array();
    // for (int i = 0; i < 6; i++)
    // {
    //     pc.printf("%d:%6.4f,", i, sens[i]);
    // }
    // pc.printf("Out:%f,F:%f\n", sensor_array.get_array_output(), sensor_array.get_filtered_output());


    //// ---- PID Troubleshoot Data ---- ////

    // float** out_arr = PID_sensor.get_terms();
    // pc.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", 
    //                 *out_arr[0], //= time_index
    //                 *out_arr[1], //= set_point
    //                 *out_arr[2], //= measurement
    //                 *out_arr[3], //= error
    //                 *out_arr[4], //= propotional
    //                 *out_arr[5], //= integrator
    //                 *out_arr[6], //= differentiator
    //                 *out_arr[7]); //= output
    
    // pc.printf("err:%.2f,", *out_arr[3]);
    // pc.printf("out:%.2f\n", *out_arr[7]);

    // pc.printf("O: %.2f | %.4f\n", *out_arr[7], *out_arr[3]);
    // pc.printf("O: %.2f | %.4f\n", *out_arr2[7], *out_arr2[3]);
    // pc.printf("D: %.2f | %.2f\n", motor_left.get_duty_cycle(), motor_right.get_duty_cycle());
    // pc.printf("S: %.4f | %.4f\n", motor_left.get_speed(), motor_right.get_speed());
}