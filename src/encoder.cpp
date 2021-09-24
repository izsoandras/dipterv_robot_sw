#include "encoder.h"

Encoder::Encoder(uint8_t gpio_pin, pcnt_unit_t unit){
    this->conf  = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = gpio_pin,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_KEEP, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
        // Set the maximum and minimum limit values to watch
        .counter_h_lim = 10000,
        .counter_l_lim = 0,
        .unit = unit,
        .channel = PCNT_CHANNEL_0,
    };
}

bool Encoder::init(){
    pcnt_unit_config(&(this->conf));
    this->pause();
    this->reset();
    return true;
}

void Encoder::resume(){
    pcnt_counter_resume(this->conf.unit);
}

void Encoder::pause(){
    pcnt_counter_pause(this->conf.unit);
}

void Encoder::reset(){
    pcnt_counter_clear(this->conf.unit);
}

int16_t Encoder::getCount(){
    int16_t count = 0;
    pcnt_get_counter_value(this->conf.unit, &count);
    return count;
}

int16_t Encoder::getCountReset(){
    int16_t count = this->getCount();
    this->reset();
    return count;
}

float Encoder::ticks2rad(uint ticks){
    return ticks * this->ticks_to_radps;
}