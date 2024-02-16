#include "mbed.h"
#include "constants.h"
#include "pin_assignments.h"
#include "bluetooth.h"
#include "motor.h"
#include "QEI.h"


class Encoder
{
protected:

    QEI qei;
    int prev_sample_time;
    int time_diff;
    volatile int curr_pulse_count;
    volatile int prev_pulse_count;
    volatile int pulse_diff;
    float rev;

public:

    Encoder(PinName CH_A, PinName CH_B): qei(CH_A, CH_B, NC, PULSE_PER_REV, QEI::X4_ENCODING) {};

    void init_time(int time_us)
    {
        prev_sample_time = time_us;
    }

    void reset(void)
    {
        qei.reset();
    }

    int get_pulse_count(void)
    {
        return qei.getPulses();   
    }

    int get_pulse_diff(void)
    {   
        curr_pulse_count = qei.getPulses();
        pulse_diff = curr_pulse_count - prev_pulse_count;
        prev_pulse_count = curr_pulse_count;
        return pulse_diff;
    }

    float get_freq(int time_us)
    {
        rev = (float)get_pulse_diff() / (float)(4 * PULSE_PER_REV);
        time_diff = (time_us - prev_sample_time);
        prev_sample_time = time_us;
        return rev * 1'000'000.0 / (float) time_diff;
    }

    float freq_to_rpm(float freq)
    {
        return freq * 60;
    }
    
    float get_speed(float freq)
    {
        return freq * WHEEL_RADIUS;
    }
};


class VectorProcessor
{
protected:
    
    int prev_sample_time;
    int time_diff;
    float angle_rad;

public:

    VectorProcessor(int time_us) 
    {
        prev_sample_time = time_us;
    };

    float get_angle(float freq_left_wheel, float freq_right_wheel, int time_us)
    {
        time_diff = (time_us - prev_sample_time);
        prev_sample_time = time_us;
        angle_rad = (freq_left_wheel - freq_right_wheel) * time_diff / (WHEEL_SEPERATION * WHEEL_RADIUS);
        return angle_rad * 180 / 3.1416;
    }
};


PwmOut LED(LED_PIN);                    // Debug LED set
Serial pc(USBTX, USBRX, 9600);          // set up serial comm with pc
Bluetooth bt(BT_TX_PIN, BT_RX_PIN);
Timer global_timer;                     // set up global program timer

int main_loop_counter = 0;      // just for fun (not important)
int last_loop_time_us = 0;      // stores the previous loop time

Motor motor_left(MOTORL_PWM_PIN, MOTORL_DIRECTION_PIN, MOTORL_BIPOLAR_PIN);
Motor motor_right(MOTORR_PWM_PIN , MOTORR_DIRECTION_PIN, MOTORR_BIPOLAR_PIN);


int main()
{
    motor_left.set_duty_cycle(0.2);
    motor_right.set_duty_cycle(0.2);

    LED.period(0.05);
    global_timer.start();           // Starts the global program timer

    while(!bt.writeable()) {};      // wait for the bluetooth to be ready

    Encoder encoder_left(MOTORL_CHA_PIN, MOTORL_CHB_PIN);
    Encoder encoder_right(MOTORR_CHA_PIN, MOTORR_CHB_PIN);
    encoder_left.init_time(global_timer.read_us());
    encoder_right.init_time(global_timer.read_us());

    VectorProcessor vp(global_timer.read_us());

    while(1)
    {
        // Checks for recieved bluetooth commands
        if (bt.data_recieved_complete()) {
            switch (bt.parse_data())
            {
                case bt.uturn:
                    bt.send_fstring("Making a U-turn");
                    // U-turn code here
                    break;
                case bt.get_run_time:
                    bt.send_fstring("Time: %.2f s", global_timer.read());
                    break;
                case bt.set_value:
                    bt.send_fstring("Duty Cycle: %f", bt.float_data1);
                    LED.write(bt.float_data1);
                    break;
                case bt.get_encoderL_pulses:
                    bt.send_fstring("L Enc: %d", main_loop_counter);
                    break;
                case bt.get_encoderR_pulses:
                    bt.send_fstring("R Enc: %d", main_loop_counter);
                    break;
                // case others:
                default:
                    bt.send_fstring("Received: %s\n", bt.get_data());
                    break;
            }
            bt.reset_rx_buffer();
        }

        pc.printf("Left Encoder Pulse Count: %d \n", encoder_left.get_pulse_count());
        pc.printf("Right Encoder Pulse Count: %d \n", encoder_right.get_pulse_count());
        float freq_l = encoder_left.get_freq(global_timer.read_us());
        float freq_r = encoder_right.get_freq(global_timer.read_us());
        pc.printf("Left Encoder Freq: %f.2 \n", freq_l);
        pc.printf("Right Encoder Freq: %f.2 \n", freq_r);

        pc.printf("Angle Calculated: %f degrees \n", vp.get_angle(freq_l, freq_r, global_timer.read_us()));


        // // simulate other part of code:
        wait_us(1'000'00);


        // End of loop
        pc.printf("Loop Time: %d us / %d \n", global_timer.read_us() - last_loop_time_us, global_timer.read_us());
        last_loop_time_us = global_timer.read_us();
        main_loop_counter++;
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