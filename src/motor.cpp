#include "motor.h"
#include "esp_log.h"

Motor::Motor(motor_config_t conf, ledc_channel_t pwm_channel, uint freq, uint8_t resolution):conf(conf),pwm_channel(pwm_channel),freq(freq),resolution(resolution){}
bool Motor::init(){
    pinMode(this->conf.dirA_pin, OUTPUT);
    pinMode(this->conf.dirB_pin, OUTPUT);
    ledcSetup(this->pwm_channel, this->freq, this->resolution);
    ledcAttachPin(this->conf.PWM_pin, this->pwm_channel);

    this->max_duty = (1<<(this->resolution)) - 1;
    //ESP_LOGW("MOTOR", "dirA: %d\ndirB: %d\nchannel: %d\nPWM: %d\nfreq: %d\nres: %d", this->conf.dirA_pin, this->conf.dirB_pin, this->pwm_channel, this->conf.PWM_pin, this->freq, this->resolution);

    this->stop();
    return true;
}

void Motor::setOutput(uint8_t dirA, uint8_t dirB, float duty_cycle){
    digitalWrite(this->conf.dirA_pin, dirA);
    digitalWrite(this->conf.dirB_pin, dirB);
    this->duty_cycle = duty_cycle;
    uint16_t pwm_duty = (uint16_t)(duty_cycle/100.0 * this->max_duty);
    ledcWrite(this->pwm_channel, pwm_duty);
    //ESP_LOGW("MOT","In: %f, Duty: %d, max: %f", duty_cycle, pwm_duty, this->max_duty);
    return;
}

void Motor::rotateCW(float duty_cycle){
    this->setOutput(HIGH, LOW, duty_cycle);
}
void Motor::rotateCCW(float duty_cycle){
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