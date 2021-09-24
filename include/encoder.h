#ifndef ENCODER_H
#define ENCODER_H

#include "driver/pcnt.h"

class Encoder{
    private:
        pcnt_config_t conf;
        float ticks_to_radps = (2*3.141592654)/(11*150); // (2*pi)/(PPR*ratio)
    public:
        Encoder(uint8_t gpio_pin, pcnt_unit_t unit);
        bool init();
        void resume();
        void pause();
        void reset();
        int16_t getCount();
        int16_t getCountReset();
        float ticks2rad(uint ticks);
};

#endif