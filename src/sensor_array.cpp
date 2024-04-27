#include "mbed.h"

#include "sensor_array.h"


float SensorArray::read(AnalogIn sensor){return 0;}


SensorArray::SensorArray(PinName sens0, PinName sens1, PinName sens2, PinName sens3, PinName sens4, PinName sens5,
            PinName led0, PinName led1, PinName led2, PinName led3, PinName led4, PinName led5, int sample_count, float detect_thresh, float angle_coefficient): 
            sample_count_(sample_count),
            detect_thresh_(detect_thresh),
            angle_coeff(angle_coefficient),
            led{led0, led1, led2, led3, led4, led5}, // Initialize led array
            sens{sens0, sens1, sens2, sens3, sens4, sens5} // Initialize sens array
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
    prev_output = 0;
    filtered_output = 0;
    prev_filtered_output = 0;
}


void SensorArray::set_all_led_on(bool status)
{
    for (int i = 0; i < 6; i++)
    {
        led[i] = status;
    }
}


void SensorArray::update(void)
{
    float sample_total[6] = {0};

    for (int i = 0; i < sample_count_; i++)
    {   
        for (int j = 0; j < 6; j++)
        {
            sample_total[j] += sens[j].read();
        }
    }

    line_detected = false;

    for (int i = 0; i < 6; i++)
    {
        sens_values[i] = sample_total[i] / sample_count_;

        float old_min = cali_min[i];
        float old_max = cali_max[i];
        float new_min = 0;
        float new_max = 1;

        // Calculate the normalized value
        float normalized_value = (sens_values[i] - old_min) / (old_max - old_min);
        
        // Map the normalized value to the new range
        float new_value = new_min + normalized_value * (new_max - new_min);
        sens_values[i] = new_value;

        // clamping the output 
        if (sens_values[i] < 0)
        {
            sens_values[i] = 0;
        }
        else if (sens_values[i] > 1)
        {
            sens_values[i] = 1;
        }

        if (sens_values[i] >= 0.4)
        {
            line_detected = true;
        }
    }

    output = angle_coeff * (sens_values[0] * coef[0] + sens_values[1] * coef[1] + sens_values[2] * coef[2] + sens_values[3] * coef[3] + sens_values[4] * coef[4] + sens_values[5] * coef[5]);
    
    // 7 Hz Pole Freq:
    // Filter coefficients b_i: [0.1802684 0.1802684]
    // Filter coefficients a_i: [0.63946321]
    
    // Filter coefficients b_i: [0.13575525 0.13575525]
    // Filter coefficients a_i: [0.7284895]

    const float LP_a0 = 0.63946321;
    const float LP_b0 = 0.1802684;
    const float LP_b1 = 0.1802684;
    
    filtered_output = (prev_filtered_output * LP_a0) + (output * LP_b0) + (prev_output * LP_b1);
    prev_filtered_output = filtered_output;
    prev_output = output;
}


bool SensorArray::is_line_detected(void)
{
    return line_detected;
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

float SensorArray::get_filtered_output(void)
{
    return filtered_output;
}

void SensorArray::calibrate_sensors(void)
{
    float sample_total[6] = {0};

    for (int i = 0; i < 100; i++)
    {   
        for (int j = 0; j < 6; j++)
        {
            sample_total[j] += sens[j].read();
        }
    }

    for (int i = 0; i < 6; i++)
    {
        sens_values[i] = sample_total[i] / 100;
        cali_min[i] = sens_values[i];
    }
}

float* SensorArray::get_calibration_constants(void)
{
    return cali_min;
}