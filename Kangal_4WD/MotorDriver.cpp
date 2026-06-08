#include "MotorDriver.h"

MotorDriver::MotorDriver(uint8_t lpwmPin, uint8_t rpwmPin)
  : _lpwmPin(lpwmPin),
    _rpwmPin(rpwmPin) {}

void MotorDriver::begin() {
  pinMode(_lpwmPin, OUTPUT);
  pinMode(_rpwmPin, OUTPUT);
  stop();
}

void MotorDriver::setSpeed(int16_t pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    analogWrite(_lpwmPin, (uint8_t)pwm);
    analogWrite(_rpwmPin, 0);
  } else if (pwm < 0) {
    analogWrite(_lpwmPin, 0);
    analogWrite(_rpwmPin, (uint8_t)(-pwm));
  } else {
    stop();
  }
}

void MotorDriver::stop() {
  analogWrite(_lpwmPin, 0);
  analogWrite(_rpwmPin, 0);
}
