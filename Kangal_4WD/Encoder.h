#pragma once

#include <Arduino.h>

#ifndef ENCODER_UPDATE_PERIOD_MS
#define ENCODER_UPDATE_PERIOD_MS 20
#endif

#ifndef ENCODER_CPR
#define ENCODER_CPR 22500
#endif

class Encoder {
public:
  Encoder(uint8_t pinA, uint8_t pinB, uint8_t id, bool reverse = false);

  void begin();
  void handleInterrupt();
  void update();

  int32_t getTicks() const;
  float getPositionRad() const;
  float getVelocityRadS() const;
  float getRPM() const;
  float getRadPerSec() const;

  uint32_t getLastDtUs() const { return _lastDtUs; }
  int32_t getLastDeltaTicks() const { return _lastDeltaTicks; }

  void reset();

private:
  uint8_t _pinA;
  uint8_t _pinB;
  uint8_t _id;
  bool _reverse;

  volatile int32_t _ticks;
  volatile uint8_t _lastAState;

  int32_t _lastTicks;
  int32_t _lastDeltaTicks;

  float _rpm;
  float _velRadS;

  uint32_t _lastTimeMs;
  uint32_t _lastUpdateUs;
  uint32_t _lastDtUs;
};
