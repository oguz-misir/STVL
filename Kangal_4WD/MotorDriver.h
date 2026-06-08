#pragma once

#include <Arduino.h>

class MotorDriver {
public:
  MotorDriver(uint8_t lpwmPin, uint8_t rpwmPin);

  void begin();
  void setSpeed(int16_t pwm);
  void stop();

private:
  uint8_t _lpwmPin;
  uint8_t _rpwmPin;
};
