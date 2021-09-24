#ifndef PINOUT_H
#define PINOUT_H

namespace pinout{
  // analog pins
  const uint8_t battery_pin = 36; // different as in PCB design, because pin 23 can't operate as analog input


  // I2C pins
  const uint8_t sda_pint = 21;
  const uint8_t scl_pint = 22;


  // motors
  const uint8_t mot1_encA = 32; // sensor on motor is dead
  const uint8_t mot1_encB = 33;
  const uint8_t mot1_dirA = 13;
  const uint8_t mot1_dirB = 15;
  const uint8_t mot1_PWM = 2;

  const uint8_t mot2_encA = 25;
  const uint8_t mot2_encB = 26;
  const uint8_t mot2_dirA = 16;
  const uint8_t mot2_dirB = 4;
  const uint8_t mot2_PWM = 17;

  const uint8_t mot3_encA = 34;
  const uint8_t mot3_encB = 35;
  const uint8_t mot3_dirA = 18;
  const uint8_t mot3_dirB = 5;
  const uint8_t mot3_PWM = 19;
}

#endif
