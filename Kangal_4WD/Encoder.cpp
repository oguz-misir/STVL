#include "Encoder.h"

#include <PinChangeInterrupt.h>

#define MAX_ENCODERS 4

static Encoder* s_instances[MAX_ENCODERS] = {nullptr};

void pcintISR0() { if (s_instances[0]) s_instances[0]->handleInterrupt(); }
void pcintISR1() { if (s_instances[1]) s_instances[1]->handleInterrupt(); }
void pcintISR2() { if (s_instances[2]) s_instances[2]->handleInterrupt(); }
void pcintISR3() { if (s_instances[3]) s_instances[3]->handleInterrupt(); }

Encoder::Encoder(uint8_t pinA, uint8_t pinB, uint8_t id, bool reverse)
  : _pinA(pinA),
    _pinB(pinB),
    _id(id),
    _reverse(reverse),
    _ticks(0),
    _lastAState(0),
    _lastTicks(0),
    _lastDeltaTicks(0),
    _rpm(0.0f),
    _velRadS(0.0f),
    _lastTimeMs(0),
    _lastUpdateUs(0),
    _lastDtUs(0) {}

void Encoder::begin() {
  pinMode(_pinA, INPUT_PULLUP);
  pinMode(_pinB, INPUT_PULLUP);

  s_instances[_id] = this;

  void (*isrList[MAX_ENCODERS])() = {pcintISR0, pcintISR1, pcintISR2, pcintISR3};

  _lastAState = (uint8_t)digitalRead(_pinA);

  // Hafif mod: sadece A kanalini dinle, yonu B kanalindan cikar.
  attachPCINT(digitalPinToPCINT(_pinA), isrList[_id], CHANGE);

  _lastTimeMs = millis();
  _lastUpdateUs = micros();
}

void Encoder::handleInterrupt() {
  const uint8_t aState = (uint8_t)digitalRead(_pinA);
  if (aState == _lastAState) {
    return;
  }
  _lastAState = aState;

  const uint8_t bState = (uint8_t)digitalRead(_pinB);
  int8_t delta = (aState == bState) ? 2 : -2;
  if (_reverse) {
    delta = -delta;
  }

  _ticks += delta;
}

void Encoder::update() {
  const uint32_t nowMs = millis();
  const uint32_t dtMs = nowMs - _lastTimeMs;
  if (dtMs < ENCODER_UPDATE_PERIOD_MS) {
    return;
  }

  noInterrupts();
  const int32_t ticks = _ticks;
  interrupts();

  const int32_t deltaTicks = ticks - _lastTicks;
  _lastDeltaTicks = deltaTicks;

  const uint32_t nowUs = micros();
  const uint32_t dtUs = nowUs - _lastUpdateUs;
  _lastUpdateUs = nowUs;
  _lastDtUs = dtUs;

  const float dtS = (dtUs > 0) ? ((float)dtUs * 1e-6f) : 0.0f;
  if (dtS > 0.0f) {
    _velRadS = (2.0f * PI) * ((float)deltaTicks / (float)ENCODER_CPR) / dtS;
  } else {
    _velRadS = 0.0f;
  }

  const float dtMin = (float)dtMs / 60000.0f;
  if (dtMin > 0.0f) {
    const float rev = (float)deltaTicks / (float)ENCODER_CPR;
    _rpm = rev / dtMin;
  } else {
    _rpm = 0.0f;
  }

  _lastTicks = ticks;
  _lastTimeMs = nowMs;
}

int32_t Encoder::getTicks() const {
  return _ticks;
}

float Encoder::getRPM() const {
  return _rpm;
}

float Encoder::getRadPerSec() const {
  return _rpm * 2.0f * PI / 60.0f;
}

float Encoder::getPositionRad() const {
  noInterrupts();
  const int32_t ticks = _ticks;
  interrupts();
  return (2.0f * PI) * (float)ticks / (float)ENCODER_CPR;
}

float Encoder::getVelocityRadS() const {
  return _velRadS;
}

void Encoder::reset() {
  noInterrupts();
  _ticks = 0;
  interrupts();

  _lastTicks = 0;
  _lastDeltaTicks = 0;
  _rpm = 0.0f;
  _velRadS = 0.0f;

  _lastAState = (uint8_t)digitalRead(_pinA);

  _lastTimeMs = millis();
  _lastUpdateUs = micros();
  _lastDtUs = 0;
}
