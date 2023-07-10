#include <iostream>
#include "pid.h"

void PIDController_Init(PIDController *pid){

    /* Clear Controller Variables */
    pid->integrator = 0.0f;
    pid->prevError  = 0.0f;

    pid->differentiator  = 0.0f;
    pid->prevMeasurement = 0.0f;

    pid->out = 0.0f;
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement) {

    /* Error Signal */
    float error = setpoint - measurement;

    /* Proportional */
    float proportional = pid->Kp * error;

    /* Integral */
    pid->integrator = pid->integrator + 
                      0.5f * pid->Kp * pid->T * (error + pid->prevError);

    if(pid->integrator > pid->limMaxInt) {

        pid->integrator = pid->limMaxInt;

    } else if(pid->integrator < pid->limMinInt) {

        pid->integrator = pid->limMinInt;

    }

    /* Differentiator (with a low pass band filter)*/
    pid->differentiator =   - (2.0f * pid->Kd * (measurement -  pid->prevMeasurement)
                            + (2.0*pid->tau - pid->T) * pid->differentiator)
                            / (2.0f * pid->tau + pid->T);


    /* Compute Controller Output */
    pid->out = proportional + pid->integrator + pid->differentiator;

    if(pid->out > pid->limMax){
        pid->out = pid->limMax;
    } else if(pid->out < pid->limMin) {
        pid->out = pid->limMin;
    }

    /* Store error and measurement for iterating */
    pid->prevError = error;
    pid->prevMeasurement = measurement;

    /* Return Controller Output */
    return pid->out;

}