#include "mbed.h"
#include "constants.h"
#include "pin_assignments.h"
#include "bluetooth.h"
#include "motor.h"
#include "QEI.h"


Serial pc(USBTX, USBRX, 9600);
Bluetooth bt(PA_11, PA_12); 


int main()
{
    Motor motor_left(MOTOR_L_PWM_PIN, MOTOR_L_DIRECTION_PIN, MOTOR_L_BIPOLAR_PIN);
    Motor motor_right(MOTOR_R_PWM_PIN , MOTOR_R_DIRECTION_PIN, MOTOR_R_BIPOLAR_PIN);
    QEI encoder_left(MOTOR_L_CHA_PIN, MOTOR_L_CHB_PIN, NC, 256, QEI::X4_ENCODING);
    QEI encoder_right(MOTOR_R_CHA_PIN, MOTOR_R_CHB_PIN, NC, 256, QEI::X4_ENCODING);

    motor_left.set_duty_cycle(0.5);
    motor_right.set_duty_cycle(0.5);

    
    int counter = 0;
    
    while (!bt.writeable()) {};

    while(1)
    {
        if (bt.data_recieved_complete()) {
            pc.printf("Received: %s\n", bt.get_data());
            bt.reset_rx_buffer();
        }

        wait_us(1000000);
        bt.send_fstring("loop number: %d \n", counter);
        counter++;

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