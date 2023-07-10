#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct {
    /* Controller Gains */
    float Kp;
    float Ki;
    float Kd;

    /* Derivative Low pass filter */
    float tau;

    /* Output Limits */
    float limMin;
    float limMax;

    /* Integrator Limits */
    float limMinInt;
    float limMaxInt;

    /* Sample Time (in seconds) */
    float T;

    /* Controller "memory" */
    float integrator;
    float prevError;            // Required for integrator
    float differentiator;
    float prevMeasurement;      // Required for differentiator

    /* Controller Output */
    float out;

} PIDController;

void PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);
float TestSystem_Update(float inp);

#endif