#ifndef MOTOR_H
#define MOTOR_H

#include "driver/ledc.h"
#include <Arduino.h>

struct motor_config_t
{
    uint8_t dirA_pin;
    uint8_t dirB_pin;
    uint8_t PWM_pin;
};


class Motor{
    private:
        motor_config_t conf;
        ledc_channel_t pwm_channel;
        uint freq;
        uint8_t resolution;
        uint8_t duty_cycle = 0;
        float max_duty;

        void setOutput(uint8_t dirA, uint8_t dirB, float duty_cycle);
    public:
        Motor(motor_config_t conf, ledc_channel_t pwm_channel, uint freq, uint8_t resolution);
        bool init();
        void rotateCW(float duty_cycle);
        void rotateCCW(float duty_cycle);
        void stop();
        uint8_t getDutyCycle();
        bool isSaturated();
        motor_config_t getConf();
};

#endif