#ifndef BATTERY_H
#include <stdint.h>
#include "pinout.h"
#define BATTERY_H

class Battery{
  private:
    uint8_t pin =  pinout::battery_pin;

    float adc_offset = 0.634;
    float adc_coeff = 11.727/2292;
    
    float voltage = 0;

    float adc2volt(uint16_t adc_value);
  public:

    Battery();
    Battery(uint8_t pin, float adc_offset, float adc_coeff);
    bool init();
    bool updateVoltage();
    float getVoltage();
};

#endif
