#include "motor.h"
#include "esp_log.h"

Motor::Motor(motor_config_t conf, ledc_channel_t pwm_channel, uint freq, uint8_t resolution):conf(conf),pwm_channel(pwm_channel),freq(freq),resolution(resolution){}
bool Motor::init(){
    pinMode(this->conf.dirA_pin, OUTPUT);
    pinMode(this->conf.dirB_pin, OUTPUT);
    ledcSetup(this->pwm_channel, this->freq, this->resolution);
    ledcAttachPin(this->conf.PWM_pin, this->pwm_channel);
    //ESP_LOGW("MOTOR", "dirA: %d\ndirB: %d\nchannel: %d\nPWM: %d\nfreq: %d\nres: %d", this->conf.dirA_pin, this->conf.dirB_pin, this->pwm_channel, this->conf.PWM_pin, this->freq, this->resolution);

    this->stop();
    return true;
}

void Motor::setOutput(uint8_t dirA, uint8_t dirB, uint8_t duty_cycle){
    digitalWrite(this->conf.dirA_pin, dirA);
    digitalWrite(this->conf.dirB_pin, dirB);
    this->duty_cycle = duty_cycle;
    uint8_t pwm_duty = (int)(duty_cycle/100.0*255);
    ledcWrite(this->pwm_channel, pwm_duty); // TODO: convert to percentage input
    //ESP_LOGW("MOT","Duty: %d", pwm_duty);
    return;
}

void Motor::rotateCW(uint8_t duty_cycle){
    this->setOutput(HIGH, LOW, duty_cycle);
}
void Motor::rotateCCW(uint8_t duty_cycle){
    this->setOutput(LOW, HIGH, duty_cycle);
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

motor_config_t Motor::getConf(){
    return conf;
}