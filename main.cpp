#include "mbed.h"
#include "constants.h"
#include "pin_assignments.h"
#include "bluetooth.h"
#include "motor.h"
#include "QEI.h"
#include "PID.h"
// #include <cstdio>


class Encoder
{
protected:

    QEI qei;
    volatile int curr_tick_count;
    volatile int prev_tick_count;
    volatile int tick_diff;
    volatile float rotational_freq;
    volatile float speed;
    volatile float rpm;

public:

    Encoder(PinName CH_A, PinName CH_B): qei(CH_A, CH_B, NC, PULSE_PER_REV, QEI::X4_ENCODING) 
    {
        prev_tick_count = 0;
    }

    void update(void)
    {
        // update pulse diff
        curr_tick_count = qei.getPulses();
        tick_diff = curr_tick_count - prev_tick_count;
        prev_tick_count = curr_tick_count;

        // update rotational freq
        rotational_freq = ((float) tick_diff / (4 * PULSE_PER_REV)) * CONTROL_UPDATE_RATE;

        // update rpm
        rpm = rotational_freq * 60;

        // update speed
        speed = 2 * PI * WHEEL_RADIUS * rotational_freq;
    }

    void reset(void)
    {
        qei.reset();
    }

    int get_tick_count(void)
    {
        return curr_tick_count;
    }

    float get_rotational_freq(void)
    {
        return rotational_freq;
    }

    float get_rpm(void)
    {
        return rpm;
    }

    float get_speed(void)
    {
        return speed;
    }
};


class VectorProcessor
{
protected:
    
    volatile float prev_cumulative_angle_deg;
    volatile float cumulative_angle_deg;
    volatile float angle_delta;
    volatile float left_set_speed;
    volatile float right_set_speed;
    float set_velocity; 
    float set_angle;
    float distance_travelled;

public:

    VectorProcessor(void) 
    {
        prev_cumulative_angle_deg = 0.0;
        cumulative_angle_deg = 0.0;
        set_velocity = 0;
        set_angle = 0;
    };
    
    void update(float tick_count_left, float tick_count_right)
    {
        cumulative_angle_deg = (float) (360 * WHEEL_RADIUS) * (tick_count_left - tick_count_right) / (WHEEL_SEPERATION * 4 * PULSE_PER_REV);
        angle_delta = cumulative_angle_deg - prev_cumulative_angle_deg;

        // angle error calculate
        float angle_error = set_angle - cumulative_angle_deg;

        // calculate speed for each motor
        float constant_speed_term = WHEEL_SEPERATION * angle_error * PI / (CONTROL_UPDATE_RATE * 360);

        left_set_speed  =  constant_speed_term + set_velocity;
        right_set_speed = -constant_speed_term + set_velocity;
    }

    float get_cumulative_angle_deg(void)
    {
        return cumulative_angle_deg;
    }

    float get_angle_delta(void)
    {
        return angle_delta;
    }

    float get_left_set_speed(void)
    {
        return left_set_speed;
    } 

    float get_right_set_speed(void)
    {
        return right_set_speed;
    }

    void set_set_velocity(float vel)
    {
        set_velocity = vel;
    }

    void set_set_angle(float ang)
    {
        set_angle = ang;
    }

    float get_distance_travelled(void)
    {
        return distance_travelled;
    }

    void reset_distance_travelled(void)
    {
        distance_travelled = 0;
    }
};


class MotorDriverBoard
{
protected:
    
    DigitalOut board_enable;
    bool enable_state;

public:

    MotorDriverBoard(PinName enable_pin): board_enable(enable_pin) {};
    MotorDriverBoard(PinName enable_pin, bool state): board_enable(enable_pin)
    {
        enable_state = state;
        board_enable.write(state);
    }

    void enable(void)
    {
        enable_state = true;
        board_enable.write(true);
    }
    
    void disable(void)
    {
        enable_state = true;
        board_enable.write(true);
    }

    bool get_enable_state(void)
    {
        return enable_state;
    }
};


typedef enum
{
    square_mode_start,
    square_mode_running,
    inactive,
} Buggy_state;


DigitalOut LED(LED_PIN);                // Debug LED set
Serial pc(USBTX, USBRX, 115200);        // set up serial comm with pc
Bluetooth bt(BT_TX_PIN, BT_RX_PIN);     
MotorDriverBoard driver_board(DRIVER_ENABLE_PIN, true);
Timer global_timer;                     // set up global program timer
Ticker control_ticker;

int main_loop_counter = 0;      // just for fun (not important)
int last_loop_time_us = 0;      // stores the previous loop time
Buggy_state buggy_state;        // stores buggy states when performing actions

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
    motor_left.set_duty_cycle(PID_motor_left.get_output());
    motor_right.set_duty_cycle(PID_motor_right.get_output());
}


int main()
{
    while (!bt.is_ready()) {};          // while bluetooth not ready, loop and do nothing

    motor_left.set_duty_cycle(0.2);
    motor_right.set_duty_cycle(0.2);

    global_timer.start();               // Starts the global program timer
    control_ticker.attach_us(&control_update_ISR, CONTROL_UPDATE_PERIOD_US);


    while (1)
    {
        /*  BLUETOOTH COMMAND HANDLING  */
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
                                break;
                            case bt.uturn:
                                // enable uturn code
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

        if (bt.is_continous() || bt.is_send_once())
        {   
            switch (bt.data_type)
            {
                case bt.pwm_duty:               // P
                    switch (bt.obj_type)
                    {
                        case bt.motor_left:
                            bt.send_fstring("PWM L: %f", motor_left.get_duty_cycle());
                            break;
                        case bt.motor_right:
                            bt.send_fstring("PWM R: %f", motor_right.get_duty_cycle());
                            break;
                        case bt.motor_both:
                            bt.send_fstring("PWM L: %.2f / R: %.2f", motor_left.get_duty_cycle(), motor_right.get_duty_cycle());
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
                            bt.send_fstring("Ticks L: %d", encoder_right.get_tick_count());
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
                            bt.send_fstring("Speed L: %f", encoder_right.get_speed());
                            break;
                        case bt.motor_both:
                            bt.send_fstring("S L: %.3f / R: %.3f", encoder_left.get_speed(), encoder_right.get_speed());
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
        /* END OF BLUEOOTH COMMAND HANDLING  */


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
            default:
                break;
        }    


        // Bunch of debug code:

        // pc.printf("Left Encoder Pulse Count: %d \n", encoder_left.get_pulse_count());
        // pc.printf("Right Encoder Pulse Count: %d \n", encoder_right.get_pulse_count());
        // float freq_l = encoder_left.get_freq(global_timer.read_us());
        // float freq_r = encoder_right.get_freq(global_timer.read_us());
        // pc.printf("Left Encoder Freq: %.2f \n", freq_l);
        // pc.printf("Right Encoder Freq: %.2f \n", freq_r);

        // float angle = vp.get_angle(freq_l, freq_r, global_timer.read_us());
        // total_angle += angle;
        // pc.printf("Angle Calculated: %.2f Degrees \n", angle);
        // pc.printf("Cumulative Angle: %.2f Degrees \n \n", total_angle);


        /*      SIMULATE FUTURE CODE:    */
        wait_us(1'000'0);

        /*       END OF LOOP      */
        last_loop_time_us = global_timer.read_us();
        main_loop_counter++;
        // pc.printf("Loop Time: %d us / %d \n \n", global_timer.read_us() - last_loop_time_us, global_timer.read_us());
    }
}



/*

class Digital_Line_Sensor{
    private:
    DigitalIn Sensor_Pin; //takes in a digital value of the voltage output from the resistor in the line sensor circuit
    public:
    Digital_Line_Sensor(PinName pin):Sensor_Pin(pin){};
    int sensor_read(){
        //reads the sensor output as an int
    };
};

class Analog_Line_Sensor{
    private:
    AnalogIn Sensor_Pin; //takes in an analog value of the voltage output from the resistor in the line sensor circuit
    public:
    Analog_Line_Sensor(PinName pin):Sensor_Pin(pin){};
    double sensor_read(){
        //reads the sensor output as a double
    };
};

class Control{
    private:
    double PID_P, PID_I, PID_D, Error; //value of the coeeficient of the PID system
                                       //Error value that is the input to the PID system
    public:
    Control(PinName p, PinName i, PinName d):PID_P(p), PID_I(i), PID_D(d){};
    double pid_control(double Line_Sensor_Value){
        //PID control function which returns a value to be used by the motors
    };
};

void bluetooth_signal_received(){
    //an ISR that is called when a bluetooth signal is recieved
};

class Battery{
    private:
    AnalogIn Battery_Voltage_Norm; //takes in an analog value to the battery voltage as a range from 0.00 to 1.00
    public:
    Battery(PinName pin):Battery_Voltage_Norm(pin){};
    float battery_SOC(){
        //returns a float value of the battery SOT to be used
    };
};



int main() {
    
 Motor motor1(PB_6,PC_7,PA_9);
 motor1.set_duty_cycle(0.5);
 motor1.set_bipolar_mode(0);
    while(1) {
        motor1.direction_mode(0);
        wait(4);
        motor1.direction_mode(1);
        wait(4);
    }
}

//This program demonstrates Nucleo board PWM signal generation using mbed
//by flashing the on-chip LED1 (PA_5)
//using PWM signal with 50% duty cycle and 0.4s period.
//#include "mbed.h"
PwmOut LED(D9); //set PA_5 "LED1" as PwmOut
int main() {
 LED.period(0.4); // 0.4 second period
 LED.write(0.50); // 50% duty cycle, relative to period
 while(1);
}
*/