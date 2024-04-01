#include "mbed.h"

#include "sensor_array.h"


float SensorArray::read(AnalogIn sensor){return 0;}


SensorArray::SensorArray(PinName sens0, PinName sens1, PinName sens2, PinName sens3, PinName sens4, PinName sens5,
            PinName led0, PinName led1, PinName led2, PinName led3, PinName led4, PinName led5, int sample_count):
            sens0_(sens0), led0_(led0),
            sens1_(sens1), led1_(led1),
            sens2_(sens2), led2_(led2),
            sens3_(sens3), led3_(led3),
            sens4_(sens4), led4_(led4),
            sens5_(sens5), led5_(led5), sample_count_(sample_count)
            {
                reset();
                set_all_led_on(false);
            };


void SensorArray::reset(void)
{
    for (int i = 0; i < sizeof(sens_values) / sizeof(sens_values[0]); i++) 
    {
        sens_values[i] = 0;
    }
    output = 0;
    angle_output = 0;
}


void SensorArray::set_all_led_on(bool status)
{
    led0_ = status;
    led1_ = status;
    led2_ = status;
    led3_ = status;
    led4_ = status;
    led5_ = status;
}


void SensorArray::update(void)
{
    float val_0_sample_total = 0;
    float val_1_sample_total = 0;
    float val_2_sample_total = 0;
    float val_3_sample_total = 0;
    float val_4_sample_total = 0;
    float val_5_sample_total = 0;

    for (int i = 0; i < sample_count_; i++)
    {            
        val_0_sample_total += sens0_.read(); 
        val_1_sample_total += sens1_.read();
        val_2_sample_total += sens2_.read();
        val_3_sample_total += sens3_.read();
        val_4_sample_total += sens4_.read();
        val_5_sample_total += sens5_.read();
    }

    sens_values[0] = val_0_sample_total / sample_count_;
    sens_values[1] = val_1_sample_total / sample_count_;
    sens_values[2] = val_2_sample_total / sample_count_;
    sens_values[3] = val_3_sample_total / sample_count_;
    sens_values[4] = val_4_sample_total / sample_count_;
    sens_values[5] = val_5_sample_total / sample_count_;

    output = (sens_values[0] * (-5) + sens_values[1] * (-3) + sens_values[2] * (-1) + sens_values[3] * (1) + sens_values[4] * (3) + sens_values[5] * (5));

    angle_output = output * (sample_count_);
}


float SensorArray::get_sens_output(int index)
{
    if ((index < sizeof(sens_values) / sizeof(sens_values[0])) && (index >= 0))
    {
        return sens_values[index]; 
    }
    return -1;
}

float* SensorArray::get_sens_output_array(void)
{
    return sens_values;
} 

float SensorArray::get_array_output(void)
{
    return output;
}

float SensorArray::get_angle_output(void)
{
    return angle_output;
}