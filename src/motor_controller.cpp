#include "motor_controller.h"

MotorControllerPID::MotorControllerPID(Motor motor, Encoder encoder, uint time_step, PIDparams pid):motor(motor), encoder(encoder), time_step(time_step),pid(pid){}
bool MotorControllerPID::init(){
    this->motor.init();
    this->encoder.init();

    //TODO: megcsinalni rendesen az init checket
    return true;
}
void MotorControllerPID::setSetpoint(float setpoint){
    this->setpoint = setpoint;
}
void MotorControllerPID::step(){
    uint ticks = this->encoder.getCountReset();
    float y = this->encoder.ticks2rad(ticks)*this->time_step;

    float e = this->setpoint - y;
    float de = this->err_prev - e;

    float u = this->pid.P * e + this->pid.I * this->err_sum + this->pid.D * de;

    if(u > 0){
        this->motor.rotateCCW((int)u);
    }else{
        this->motor.rotateCW((int)u);
    }

    if(!this->motor.isSaturated()){
        this->err_sum += e;
    }
    
    this->err_prev = e;
}