#pragma once

#include "mbed.h"

class PID
{
private:

    float Kp; 
    float Ki; 
    float Kd; 
    float limMin; 
    float limMax; 
    float limMinInt; 
    float limMaxInt; 
    float integrator; 
    float prevError; 
    float differentiator; 
    float position;
    float prevposition; 
    float output; 
    float setposition;

public:

    PID(
        float Kp, 
        float Ki, 
        float Kd, 
        float limMin, 
        float limMax, 
        float limMinInt, 
        float limMaxInt,
        float setposition);

    void set(   
        float Kp1, 
        float Ki1, 
        float Kd1, 
        float limMin1, 
        float limMax1, 
        float limMinInt1, 
        float limMaxInt1);


    void reset();


    float getoutput(float setposition, float position);
};


/*
int main(){
    
    PIDmotorController leftmotor(0,0,0,0,0,0,0,0);
    PIDmotorController rightmotor(0,0,0,0,0,0,0,0);
    PIDmotorController angle(0,0,0,0,0,0,0,0);
    angle.set(1.2,0.5,0.05,-200,200,-200,200);




}
*/