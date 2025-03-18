#include "Esp32_Cytron_MDxx.h"

ESP32_CYTRON_MD ::ESP32_CYTRON_MD(uint8_t _pwm_pin, uint8_t _dir_pin, uint32_t _freq, uint8_t _res)
  : pwm_pin(_pwm_pin), dir_pin(_dir_pin), freq(_freq), resolution(_res) {}

void ESP32_CYTRON_MD ::begin() {
  pinMode(dir_pin, OUTPUT);
  ledcAttach(pwm_pin, freq, resolution);
}

void ESP32_CYTRON_MD ::set_duty(int16_t _duty) {
  if (_duty >= 0) {
    digitalWrite(dir_pin, 1);
    ledcWrite(pwm_pin, _duty);
  } else {
    digitalWrite(dir_pin, 0);
    ledcWrite(pwm_pin, -1 * _duty);
  }
}