#ifndef _ESP32_CYTRON_MDXX_H__
#define _ESP32_CYTRON_MDXX_H__

#include "stdint.h"
#include "Arduino.h"
#include "esp32-hal-ledc.h"

class ESP32_CYTRON_MD {
  private:

  uint8_t pwm_pin;
  uint8_t dir_pin;
  uint32_t freq;
  uint8_t resolution;

  public:

  ESP32_CYTRON_MD(uint8_t _pwm_pin, uint8_t _dir_pin, uint32_t _freq = 1000, uint8_t _res = 14);

  void begin();

  void set_duty(int16_t _duty);

};

#endif