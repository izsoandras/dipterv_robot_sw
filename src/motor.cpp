#include "motor.h"

Motor::Motor(motor_config_t conf, ledc_channel_t pwm_channel, uint freq, uint8_t resolution):conf(conf),pwm_channel(pwm_channel),freq(freq),resolution(resolution){}
bool Motor::init(){
    pinMode(this->conf.PWM_pin, OUTPUT);
    pinMode(this->conf.dirA_pin, OUTPUT);
    pinMode(this->conf.dirB_pin, OUTPUT);
    ledcSetup(this->conf.PWM_pin, this->freq, this->resolution);
    ledcAttachPin(this->conf.PWM_pin, this->pwm_channel);
}

void Motor::setOutput(bool dirA, bool dirB, uint8_t duty_cycle_percent){
    digitalWrite(this->conf.dirA_pin, dirA);
    digitalWrite(this->conf.dirB_pin, dirB);
    this->duty_cycle = duty_cycle_percent;
    ledcWrite(this->pwm_channel, duty_cycle_percent); // TODO: convert to percentage input
}

void Motor::rotateCW(uint8_t duty_cycle){
    this->setOutput(HIGH, LOW, duty_cycle);    // TODO: check direction
                                            
}
void Motor::rotateCCW(uint8_t duty_cycle){
    this->setOutput(LOW, HIGH, duty_cycle);    // TODO: check direction
                                            // TODO: convert to percentage input
}
void Motor::stop(){
    this->setOutput(LOW, LOW, 0); // TODO: check if this is slow decay
}

uint8_t Motor::getDutyCycle(){
    return this-> duty_cycle;
}

bool Motor::isSaturated(){
    return this->duty_cycle >= 100;
}