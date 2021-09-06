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

        void setOutput(uint8_t dirA, uint8_t dirB, uint8_t duty_cycle);
    public:
        Motor(motor_config_t conf, ledc_channel_t pwm_channel, uint freq, uint8_t resolution);
        bool init();
        void rotateCW(uint8_t duty_cycle);
        void rotateCCW(uint8_t duty_cycle);
        void stop();
        uint8_t getDutyCycle();
        bool isSaturated();
};

#endif