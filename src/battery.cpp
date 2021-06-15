#include <Arduino.h>
#include "battery.h"

Battery::Battery(){

}

Battery::Battery(uint8_t pin, float adc_offset, float adc_coeff){
  this->pin = pin;
  this->adc_offset = adc_offset;
  this->adc_coeff = adc_coeff;
}

float Battery::adc2volt(uint16_t adc_value){
  return adc_value * this->adc_coeff + this->adc_offset;
}

bool Battery::init(){
  pinMode(this->pin, INPUT);
  return true;
}

bool Battery::updateVoltage(){
  this->voltage = adc2volt(analogRead(this->pin));
  return true;
}

float Battery::getVoltage(){
  return this->voltage;
}
