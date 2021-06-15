#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "motor.h"
#include "encoder.h"

struct PIDparams{
    float P;
    float I;
    float D;
};

class MotorControllerPID{
    private:
        Motor motor;
        Encoder encoder;
        uint time_step; // period time of the control circle [ms]
        PIDparams pid;
        float err_sum = 0;
        float err_prev = 0;
        float setpoint;
    public:
        MotorControllerPID(Motor motor, Encoder encoder, uint time_step, PIDparams pid);
        bool init();
        void setSetpoint(float setpoint);
        void step();
};

#endif