#include <Arduino.h>

#include "MotorDriver.h"
#include "Encoder.h"

// ===============================
// MOTOR PINLERI
// ===============================
#define PIN_MOTOR_FR_LPWM   10
#define PIN_MOTOR_FR_RPWM   9

#define PIN_MOTOR_RR_LPWM   11
#define PIN_MOTOR_RR_RPWM   12

#define PIN_MOTOR_FL_LPWM   2
#define PIN_MOTOR_FL_RPWM   3

#define PIN_MOTOR_RL_LPWM   6
#define PIN_MOTOR_RL_RPWM   5

MotorDriver motorFR(PIN_MOTOR_FR_LPWM, PIN_MOTOR_FR_RPWM);
MotorDriver motorRR(PIN_MOTOR_RR_LPWM, PIN_MOTOR_RR_RPWM);
MotorDriver motorFL(PIN_MOTOR_FL_LPWM, PIN_MOTOR_FL_RPWM);
MotorDriver motorRL(PIN_MOTOR_RL_LPWM, PIN_MOTOR_RL_RPWM);

static const int PWM_MAX = 255;
static const int DRIVE_SPEED_LIMIT_PCT = 60; // ileri/geri hiz tavani mevcut maksimumun %60'i
static const int DRIVE_MAX_PWM = (PWM_MAX * DRIVE_SPEED_LIMIT_PCT) / 100;
static const float WHEEL_RADIUS_CM = 19.0f;
static const float WHEEL_CIRCUMFERENCE_CM = 2.0f * PI * WHEEL_RADIUS_CM;
static const bool MOTOR_FR_REVERSE = true;
static const bool MOTOR_RR_REVERSE = true;
static const bool MOTOR_FL_REVERSE = false;
static const bool MOTOR_RL_REVERSE = false;
static const int PAIR_DRIVE_MOTOR_SIGN = -1; // RC ve cift-teker seri surusu mevcut yon hissini korusun
static const int SERIAL_PWM_SIGN = -1; // seri cift-teker komutlari tek-teker test ile ayni yone gitsin

// ===============================
// ENCODER PINLERI
// ===============================
#define PIN_ENCODER_FR_A    A10
#define PIN_ENCODER_FR_B    A11

#define PIN_ENCODER_RR_A    A9
#define PIN_ENCODER_RR_B    A8

#define PIN_ENCODER_FL_A    A12
#define PIN_ENCODER_FL_B    A13

#define PIN_ENCODER_RL_A    A15
#define PIN_ENCODER_RL_B    A14

static const bool FR_REVERSE = false;
static const bool RR_REVERSE = false;
static const bool FL_REVERSE = false;
static const bool RL_REVERSE = false;
static const bool RR_ENCODER_ENABLED = true;

Encoder encFR(PIN_ENCODER_FR_A, PIN_ENCODER_FR_B, 0, FR_REVERSE);
Encoder encRR(PIN_ENCODER_RR_A, PIN_ENCODER_RR_B, 1, RR_REVERSE);
Encoder encFL(PIN_ENCODER_FL_A, PIN_ENCODER_FL_B, 2, FL_REVERSE);
Encoder encRL(PIN_ENCODER_RL_A, PIN_ENCODER_RL_B, 3, RL_REVERSE);

// ===============================
// RC PINLERI
// rc_test.ino ile ayni pinler kullanildi
// ===============================
#define CH1_PIN 18
#define CH2_PIN 19
#define CH3_PIN 20

static const bool RC_STEERING_REVERSE = false;
static const bool RC_THROTTLE_REVERSE = false;

static const int RC_PULSE_MIN_US = 1000;
static const int RC_PULSE_MID_US = 1500;
static const int RC_PULSE_MAX_US = 2000;
static const int RC_STEERING_CENTER_US = 1500;
static const int RC_THROTTLE_CENTER_US = 1500;
static const int RC_ENABLE_CENTER_US = 1500;
static const int RC_STEERING_READY_DEADBAND_US = 35;
static const int RC_STEERING_PIVOT_DEADBAND_US = 8;
static const int RC_STEERING_DRIVE_DEADBAND_US = 20;
static const int RC_STEERING_PIVOT_START_US = 30;
static const int RC_STEERING_PIVOT_KEEP_US = 12;
static const int RC_THROTTLE_DEADBAND_US = 24;
static const int RC_THROTTLE_IDLE_EXIT_US = 36;
static const int RC_THROTTLE_IDLE_ENTER_US = 18;
static const int RC_MIN_EFFECTIVE_PWM = 12;
static const int RC_ENABLE_ACTIVE_DELTA_US = 200;
static const int RC_ENABLE_ON_THRESHOLD_1000 = 650;
static const int RC_ENABLE_OFF_THRESHOLD_1000 = 350;
static const int RC_STEERING_PIVOT_GAIN_PCT = 220;
static const int RC_STEERING_DRIVE_GAIN_PCT = 170;
static const int RC_THROTTLE_GAIN_PCT = 200;
static const int RC_PIVOT_MAX_PWM = 180;
static const int RC_DRIVE_SELECT_MARGIN_PWM = 24;
static const int RC_DRIVE_MIN_INNER_THROTTLE_PCT = 15; // agresif gazli donuste ic taraf daha fazla dussun
static const int RC_DRIVE_INNER_AGGRESSION_PCT = 175; // 100 = klasik ic teker yavaslatma, >100 ic tarafi daha sert kirar
static const int RC_DRIVE_OUTER_BOOST_PCT = 30; // dis tarafta steer'e bagli ek hiz farki olustur
static const int RC_DRIVE_COUNTER_THROTTLE_MAX_PWM = 45; // gazli donuste ters yone sadece cok dusuk hizda izin ver
static const int RC_DRIVE_COUNTER_STEER_MIN_PCT = 88; // ic tarafin ters yone dusmesi icin steer neredeyse sonda olmali
static const int RC_INPUT_FILTER_STEP_US = 12;
static const int RC_SAFETY_THRESHOLD_1000 = 500;
static const int RC_THROTTLE_CENTER_CAL_WINDOW_US = 20;
static const int RC_STEERING_FILTER_US_PER_MS = 60;
static const int RC_SAFETY_FILTER_US_PER_MS = 120;
static const int RC_THROTTLE_FILTER_US_PER_MS = 120;
static const bool RC_INPUT_FILTER_ENABLED = false;
static const int RC_CALIBRATION_STABLE_SPAN_US = 25;
static const uint32_t RC_NEUTRAL_CONFIRM_MS = 300;
static const uint32_t RC_PIVOT_CONFIRM_MS = 60;
static const uint32_t RC_SIGNAL_TIMEOUT_US = 60000;
static const uint32_t RC_SIGNAL_LOSS_HOLD_MS = 120;
static const uint32_t RC_REARM_AFTER_SIGNAL_LOSS_MS = 1000;
static const uint32_t RC_CALIBRATION_MS = 1500;
static const uint16_t RC_MIN_CALIBRATION_SAMPLES = 20;
static const uint32_t RC_DIRECT_PULSE_TIMEOUT_US = 25000;
static const uint32_t RC_DIRECT_SAMPLE_RETRY_MS = 40;
static const uint32_t RC_ENABLE_AUTO_ARM_MS = 800;

volatile uint32_t ch1RiseUs = 0;
volatile uint32_t ch2RiseUs = 0;
volatile uint32_t ch3RiseUs = 0;

volatile uint16_t ch1PulseUs = RC_PULSE_MID_US;
volatile uint16_t ch2PulseUs = RC_PULSE_MID_US;
volatile uint16_t ch3PulseUs = RC_PULSE_MID_US;

volatile uint32_t ch1LastPulseUs = 0;
volatile uint32_t ch2LastPulseUs = 0;
volatile uint32_t ch3LastPulseUs = 0;

static int rcSteeringCenterUs = RC_STEERING_CENTER_US;
static int rcCh2CenterUs = RC_THROTTLE_CENTER_US;
static int rcCh3CenterUs = RC_ENABLE_CENTER_US;
static int rcFilteredCh1Us = RC_STEERING_CENTER_US;
static int rcFilteredCh2Us = RC_THROTTLE_CENTER_US;
static int rcFilteredCh3Us = RC_ENABLE_CENTER_US;
static bool rcNeutralReady = false;
static uint32_t rcNeutralSinceMs = 0;
static uint32_t rcSignalLostSinceMs = 0;
static uint32_t rcSignalLossPendingSinceMs = 0;
static uint32_t rcLastGoodSignalMs = 0;
static bool rcEnableActiveLatched = false;
static bool rcPivotActive = false;
static bool rcThrottleIdleLatched = true;
static bool rcEnableReleasedSinceReady = false;
static bool rcControlArmed = false;
static uint32_t rcEnableActiveSinceReadyMs = 0;
static uint32_t rcPivotRequestSinceMs = 0;
static uint32_t rcLastFilterUpdateUs = 0;
static uint32_t rcLastDirectSampleMs = 0;
static int rcSteeringReadyDeadbandUs = RC_STEERING_READY_DEADBAND_US;
static int rcSteeringDriveDeadbandUs = RC_STEERING_DRIVE_DEADBAND_US;
static int rcSteeringPivotStartUs = RC_STEERING_PIVOT_START_US;
static int rcSteeringPivotKeepUs = RC_STEERING_PIVOT_KEEP_US;
static int rcThrottleNeutralDeadbandUs = RC_THROTTLE_DEADBAND_US;
static int rcThrottleIdleExitUs = RC_THROTTLE_IDLE_EXIT_US;
static int rcThrottleIdleEnterUs = RC_THROTTLE_IDLE_ENTER_US;
static bool rcSteeringPivotLatched = false;

enum AxisMode : uint8_t {
  AXIS_MODE_CENTERED = 0,
  AXIS_MODE_LOW_REST = 1,
  AXIS_MODE_HIGH_REST = 2
};

enum DriveChannel : uint8_t {
  DRIVE_CHANNEL_CH2 = 2,
  DRIVE_CHANNEL_CH3 = 3
};

enum SerialWheelId : uint8_t {
  SERIAL_WHEEL_FL = 0,
  SERIAL_WHEEL_FR = 1,
  SERIAL_WHEEL_RL = 2,
  SERIAL_WHEEL_RR = 3,
  SERIAL_WHEEL_NONE = 255
};

enum SerialCommandType : uint8_t {
  SERIAL_CMD_INVALID = 0,
  SERIAL_CMD_STOP = 1,
  SERIAL_CMD_PAIR_PWM = 2,
  SERIAL_CMD_WHEEL_RPM = 3
};

struct SerialParsedCommand {
  SerialCommandType type;
  int pwmL;
  int pwmR;
  SerialWheelId wheelId;
  int targetRpm;
};

static AxisMode rcCh2Mode = AXIS_MODE_CENTERED;
static AxisMode rcCh3Mode = AXIS_MODE_CENTERED;
static const DriveChannel RC_DEFAULT_DRIVE_CHANNEL = DRIVE_CHANNEL_CH3;
static DriveChannel rcActiveDriveChannel = RC_DEFAULT_DRIVE_CHANNEL;
static DriveChannel rcEnableChannel = DRIVE_CHANNEL_CH2;

// ===============================
// ZAMANLAMALAR
// ===============================
static const uint32_t JS_TELEMETRY_MS   = 80;   // 12.5 Hz
static const uint32_t RC_TELEMETRY_MS   = 200;  // 5 Hz
static const uint32_t RAW_TELEMETRY_MS  = 250;  // 4 Hz
static const uint32_t MODE_TELEMETRY_MS = 500;  // 2 Hz
static const uint32_t CMD_TIMEOUT_MS = 400;  // RC sinyali yoksa dur
static const uint32_t SERIAL_CMD_TIMEOUT_MS = 400;
static const int OUTPUT_ACCEL_PWM_PER_MS = 2;
static const int OUTPUT_DECEL_PWM_PER_MS = 1;
static const int OUTPUT_REVERSE_DECEL_PWM_PER_MS = 3;
static const int OUTPUT_REVERSE_ACCEL_PWM_PER_MS = 2;
static const uint32_t OUTPUT_DRIVE_STOP_PWM_STEP_INTERVAL_MS = 10;
static const uint32_t OUTPUT_PIVOT_STOP_PWM_STEP_INTERVAL_MS = 10;
static const int OUTPUT_EMERGENCY_STOP_PWM_PER_MS = 8; // sinyal kaybi / guvenlikte hizli durus
static const bool OUTPUT_SOFT_STOP_ENABLED = true;
static const uint32_t OUTPUT_REVERSE_HOLD_MS = 0;
static const uint32_t OUTPUT_REVERSE_NEUTRAL_SETTLE_MS = 0;
static const uint32_t OUTPUT_REVERSE_MAX_WAIT_MS = 160;
static const float OUTPUT_REVERSE_RELEASE_RPM = 30.0f;
static const bool ENABLE_ENCODER_MONITORING = true;
static const bool ENABLE_WHEEL_SPEED_BALANCE = true;
static const int WHEEL_BALANCE_MIN_ACTIVE_PWM = 45;
static const float WHEEL_BALANCE_MIN_ACTIVE_RPM = 3.0f;
static const float WHEEL_BALANCE_PAIR_DEADBAND_RPM = 1.5f;
static const float WHEEL_BALANCE_SIDE_DEADBAND_RATIO = 0.03f;
static const float WHEEL_BALANCE_PAIR_KP = 0.8f;    // pwm / rpm
static const float WHEEL_BALANCE_SIDE_KP = 160.0f;  // pwm / normalized response error
static const int WHEEL_BALANCE_PAIR_MAX_TRIM_PWM = 26;
static const int WHEEL_BALANCE_SIDE_MAX_TRIM_PWM = 18;
static const int WHEEL_BALANCE_TRIM_SLEW_PWM_PER_MS = 1;
static const bool JS_COMPACT_TELEMETRY = true;
static const bool ENABLE_RAW_ENCODER_TELEMETRY = false;
static const int SERIAL_JS_BUFFER_BYTES = 160;
static const int SERIAL_RC_BUFFER_BYTES = 48;
static const int SERIAL_RAW_BUFFER_BYTES = 96;
static const int SERIAL_WTEST_BUFFER_BYTES = 96;
static const int SERIAL_WHEEL_TEST_MAX_RPM = 180;
static const int SERIAL_WHEEL_TEST_MAX_PWM = PWM_MAX;
static const int SERIAL_WHEEL_TEST_MIN_ACTIVE_PWM = 45;
static const int SERIAL_WHEEL_TEST_PWM_SLEW_PER_MS = 6;
static const float SERIAL_WHEEL_TEST_FEEDFORWARD_PWM_PER_RPM = 2.8f;
static const float SERIAL_WHEEL_TEST_KP = 1.0f;
static const float SERIAL_WHEEL_TEST_KI = 1.4f;
static const float SERIAL_WHEEL_TEST_INTEGRAL_LIMIT = 120.0f;
static const uint32_t SERIAL_WHEEL_CMD_TIMEOUT_MS = 0;  // 0 = STOP gelene kadar aktif tut

static uint32_t lastJsTeleMs = 0;
static uint32_t lastRcTeleMs = 0;
static uint32_t lastRawTeleMs = 0;
static uint32_t lastModeTeleMs = 0;
static uint32_t seq = 0;

// Son PWM komutu
static int pwmL_cmd = 0;
static int pwmR_cmd = 0;
static int serialPwmL_cmd = 0;
static int serialPwmR_cmd = 0;
static int pwmL_out = 0;
static int pwmR_out = 0;
static int rcDebugThrottlePwm = 0;
static int rcDebugSteeringPwm = 0;
static int rcDebugPwmLCmd = 0;
static int rcDebugPwmRCmd = 0;
static uint32_t outputActiveStopPwmStepIntervalMs = OUTPUT_DRIVE_STOP_PWM_STEP_INTERVAL_MS;
static uint32_t lastCmdMs = 0;
static uint32_t serialLastCmdMs = 0;
static uint32_t lastOutputUpdateMs = 0;
static uint32_t pwmLReverseHoldUntilMs = 0;
static uint32_t pwmRReverseHoldUntilMs = 0;
static uint32_t pwmLReverseWaitStartMs = 0;
static uint32_t pwmRReverseWaitStartMs = 0;
static uint32_t pwmLReverseNeutralUntilMs = 0;
static uint32_t pwmRReverseNeutralUntilMs = 0;
static uint32_t pwmLZeroStopCarryMs = 0;
static uint32_t pwmRZeroStopCarryMs = 0;
static bool pwmLReverseRestartActive = false;
static bool pwmRReverseRestartActive = false;
static int wheelBalanceLeftPairTrim = 0;
static int wheelBalanceRightPairTrim = 0;
static int wheelBalanceSideTrim = 0;
static bool serialOverrideActive = false;
static bool serialWheelModeActive = false;
static SerialWheelId serialWheelId = SERIAL_WHEEL_NONE;
static int serialWheelTargetRpmCmd = 0;
static int serialWheelPwmCmd = 0;
static int serialWheelPwmOut = 0;
static float serialWheelMeasuredRpm = 0.0f;
static float serialWheelIntegral = 0.0f;
static uint32_t serialWheelLastControlMs = 0;
static char serialRxLine[64];
static uint8_t serialRxLen = 0;
static uint8_t rawFlA = 0;
static uint8_t rawFlB = 0;
static uint8_t rawFrA = 0;
static uint8_t rawFrB = 0;
static uint8_t rawRlA = 0;
static uint8_t rawRlB = 0;
static uint8_t rawRrA = 0;
static uint8_t rawRrB = 0;
static uint32_t rawFlEdges = 0;
static uint32_t rawFrEdges = 0;
static uint32_t rawRlEdges = 0;
static uint32_t rawRrEdges = 0;

// ===============================
// Yardimcilar
// ===============================
static inline int clampi(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline int maxi(int a, int b) {
  return (a > b) ? a : b;
}

static inline float absf_local(float x) {
  return (x >= 0.0f) ? x : -x;
}

static inline long floatToSigned10(float value) {
  return (long)((value >= 0.0f) ? (value * 10.0f + 0.5f) : (value * 10.0f - 0.5f));
}

static inline float rpmToWheelSpeedCmS(float rpm) {
  return (rpm * WHEEL_CIRCUMFERENCE_CM) / 60.0f;
}

static inline float clampf_local(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline float applyDeadbandFloat(float value, float deadband) {
  if (value > deadband) return value - deadband;
  if (value < -deadband) return value + deadband;
  return 0.0f;
}

static int rampToward(int current, int target, int maxStep);

static inline int applyDeadband(int value, int deadband) {
  if (value > deadband) return value - deadband;
  if (value < -deadband) return value + deadband;
  return 0;
}

static inline int zeroSmallPwm(int value, int threshold) {
  if (value > -threshold && value < threshold) {
    return 0;
  }
  return value;
}

static inline int applyGainPct(int value, int gainPct) {
  return clampi((int)(((long)value * gainPct) / 100L), -PWM_MAX, PWM_MAX);
}

static inline int filterTowardInt(int current, int target, int maxStep) {
  if (target > current + maxStep) return current + maxStep;
  if (target < current - maxStep) return current - maxStep;
  return target;
}

static inline int filterTowardTimed(int current, int target, uint32_t deltaUs, int filterUsPerMs) {
  const int maxStep = clampi((int)(((unsigned long)deltaUs * (unsigned long)filterUsPerMs) / 1000UL), 1, 2000);
  return filterTowardInt(current, target, maxStep);
}

static inline bool serialHasRoom(int minFreeBytes) {
  return Serial.availableForWrite() >= minFreeBytes;
}

static inline bool serialWriteLineIfFits(const char* line, size_t length) {
  if (!line || length == 0) {
    return true;
  }
  if (!serialHasRoom((int)length)) {
    return false;
  }
  Serial.write((const uint8_t*)line, length);
  return true;
}

static bool readSerialLineNonBlocking(char* out, size_t outSz) {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      serialRxLine[serialRxLen] = '\0';
      strncpy(out, serialRxLine, outSz);
      out[outSz - 1] = '\0';
      serialRxLen = 0;
      return true;
    }

    if (serialRxLen < sizeof(serialRxLine) - 1) {
      serialRxLine[serialRxLen++] = c;
    } else {
      serialRxLen = 0;
    }
  }

  return false;
}

static inline char asciiUpper(char c) {
  if (c >= 'a' && c <= 'z') {
    return (char)(c - ('a' - 'A'));
  }
  return c;
}

static inline bool isTokenTerminator(char c) {
  return c == '\0' || c == ' ' || c == '\t' || c == ',';
}

static const char* getSerialWheelName(SerialWheelId wheelId) {
  switch (wheelId) {
    case SERIAL_WHEEL_FL: return "FL";
    case SERIAL_WHEEL_FR: return "FR";
    case SERIAL_WHEEL_RL: return "RL";
    case SERIAL_WHEEL_RR: return "RR";
    default: return "NONE";
  }
}

static float getSerialWheelMeasuredRpm(SerialWheelId wheelId) {
  if (!ENABLE_ENCODER_MONITORING) {
    return 0.0f;
  }

  switch (wheelId) {
    case SERIAL_WHEEL_FL: return encFL.getRPM();
    case SERIAL_WHEEL_FR: return encFR.getRPM();
    case SERIAL_WHEEL_RL: return encRL.getRPM();
    case SERIAL_WHEEL_RR: return RR_ENCODER_ENABLED ? encRR.getRPM() : 0.0f;
    default: return 0.0f;
  }
}

static void resetSerialWheelControl() {
  serialWheelModeActive = false;
  serialWheelId = SERIAL_WHEEL_NONE;
  serialWheelTargetRpmCmd = 0;
  serialWheelPwmCmd = 0;
  serialWheelPwmOut = 0;
  serialWheelMeasuredRpm = 0.0f;
  serialWheelIntegral = 0.0f;
  serialWheelLastControlMs = 0;
  wheelBalanceLeftPairTrim = 0;
  wheelBalanceRightPairTrim = 0;
  wheelBalanceSideTrim = 0;
}

static bool tryParseSerialWheelCmd(const char* line,
                                   SerialWheelId& outWheelId,
                                   int& outTargetRpm) {
  if (!line) {
    return false;
  }

  const char c0 = asciiUpper(line[0]);
  const char c1 = asciiUpper(line[1]);
  if (!isTokenTerminator(line[2])) {
    return false;
  }

  if (c0 == 'F' && c1 == 'L') {
    outWheelId = SERIAL_WHEEL_FL;
  } else if (c0 == 'F' && c1 == 'R') {
    outWheelId = SERIAL_WHEEL_FR;
  } else if (c0 == 'R' && c1 == 'L') {
    outWheelId = SERIAL_WHEEL_RL;
  } else if (c0 == 'R' && c1 == 'R') {
    outWheelId = SERIAL_WHEEL_RR;
  } else {
    return false;
  }

  line += 2;
  while (*line == ' ' || *line == '\t' || *line == ',') {
    line++;
  }

  int targetRpm = 0;
  if (sscanf(line, "%d", &targetRpm) != 1) {
    return false;
  }

  outTargetRpm = clampi(targetRpm,
                        -SERIAL_WHEEL_TEST_MAX_RPM,
                        SERIAL_WHEEL_TEST_MAX_RPM);
  return true;
}

static bool parseSerialCommand(const char* line, SerialParsedCommand& outCmd) {
  outCmd.type = SERIAL_CMD_INVALID;
  outCmd.pwmL = 0;
  outCmd.pwmR = 0;
  outCmd.wheelId = SERIAL_WHEEL_NONE;
  outCmd.targetRpm = 0;

  while (*line == ' ' || *line == '\t') {
    line++;
  }
  if (*line == '\0') {
    return false;
  }

  if (asciiUpper(line[0]) == 'S' &&
      asciiUpper(line[1]) == 'T' &&
      asciiUpper(line[2]) == 'O' &&
      asciiUpper(line[3]) == 'P' &&
      isTokenTerminator(line[4])) {
    outCmd.type = SERIAL_CMD_STOP;
    return true;
  }

  SerialWheelId wheelId = SERIAL_WHEEL_NONE;
  int targetRpm = 0;
  if (tryParseSerialWheelCmd(line, wheelId, targetRpm)) {
    outCmd.type = SERIAL_CMD_WHEEL_RPM;
    outCmd.wheelId = wheelId;
    outCmd.targetRpm = targetRpm;
    return true;
  }

  if (asciiUpper(line[0]) == 'P' &&
      asciiUpper(line[1]) == 'W' &&
      asciiUpper(line[2]) == 'M' &&
      isTokenTerminator(line[3])) {
    line += 3;
    while (*line == ' ' || *line == '\t') {
      line++;
    }
  }

  int l = 0;
  int r = 0;
  int n = sscanf(line, "%d %d", &l, &r);
  if (n == 2) {
    outCmd.type = SERIAL_CMD_PAIR_PWM;
    outCmd.pwmL = clampi(l * SERIAL_PWM_SIGN, -PWM_MAX, PWM_MAX);
    outCmd.pwmR = clampi(r * SERIAL_PWM_SIGN, -PWM_MAX, PWM_MAX);
    return true;
  }

  n = sscanf(line, "%d,%d", &l, &r);
  if (n == 2) {
    outCmd.type = SERIAL_CMD_PAIR_PWM;
    outCmd.pwmL = clampi(l * SERIAL_PWM_SIGN, -PWM_MAX, PWM_MAX);
    outCmd.pwmR = clampi(r * SERIAL_PWM_SIGN, -PWM_MAX, PWM_MAX);
    return true;
  }

  n = sscanf(line, "%d", &l);
  if (n == 1) {
    outCmd.type = SERIAL_CMD_PAIR_PWM;
    outCmd.pwmL = clampi(l * SERIAL_PWM_SIGN, -PWM_MAX, PWM_MAX);
    outCmd.pwmR = clampi(l * SERIAL_PWM_SIGN, -PWM_MAX, PWM_MAX);
    return true;
  }

  return false;
}

static void pollSerialCommands() {
  char line[64];
  while (readSerialLineNonBlocking(line, sizeof(line))) {
    SerialParsedCommand cmd;
    if (!parseSerialCommand(line, cmd)) {
      continue;
    }

    const uint32_t nowMs = millis();
    serialLastCmdMs = nowMs;

    if (cmd.type == SERIAL_CMD_STOP) {
      serialPwmL_cmd = 0;
      serialPwmR_cmd = 0;
      serialOverrideActive = false;
      resetSerialWheelControl();
      continue;
    }

    if (cmd.type == SERIAL_CMD_PAIR_PWM) {
      serialPwmL_cmd = cmd.pwmL;
      serialPwmR_cmd = cmd.pwmR;
      resetSerialWheelControl();
      serialOverrideActive = true;
      continue;
    }

    if (cmd.type == SERIAL_CMD_WHEEL_RPM) {
      serialPwmL_cmd = 0;
      serialPwmR_cmd = 0;
      resetSerialWheelControl();
      if (cmd.targetRpm == 0) {
        serialOverrideActive = false;
        continue;
      }

      serialWheelModeActive = true;
      serialWheelId = cmd.wheelId;
      serialWheelTargetRpmCmd = cmd.targetRpm;
      serialWheelLastControlMs = nowMs;
      serialOverrideActive = true;
    }
  }
}

static bool isSerialOverrideActive(uint32_t nowMs) {
  if (!serialOverrideActive) {
    return false;
  }

  const uint32_t timeoutMs =
      serialWheelModeActive ? SERIAL_WHEEL_CMD_TIMEOUT_MS : SERIAL_CMD_TIMEOUT_MS;
  if (timeoutMs > 0 &&
      (nowMs - serialLastCmdMs) > timeoutMs) {
    serialOverrideActive = false;
    resetSerialWheelControl();
    return false;
  }

  return true;
}

static bool isRcModeEnabledNow() {
  refreshRcSignalsDirect(false);

  uint16_t ch1Us = 0;
  uint16_t ch2Us = 0;
  uint16_t ch3Us = 0;
  uint32_t ch1StampUs = 0;
  uint32_t ch2StampUs = 0;
  uint32_t ch3StampUs = 0;

  readRcSnapshot(ch1Us, ch2Us, ch3Us, ch1StampUs, ch2StampUs, ch3StampUs);

  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();
  const bool ch1Valid = isSignalFresh(nowUs, ch1StampUs);
  const bool ch2Valid = isSignalFresh(nowUs, ch2StampUs);
  const bool ch3Valid = isSignalFresh(nowUs, ch3StampUs);
  const bool allValid = ch1Valid && ch2Valid && ch3Valid;
  if (allValid) {
    rcLastGoodSignalMs = nowMs;
  }

  const bool withinSignalGrace =
      rcLastGoodSignalMs != 0 &&
      (nowMs - rcLastGoodSignalMs) <= RC_SIGNAL_LOSS_HOLD_MS;
  const bool enableValid =
      (rcEnableChannel == DRIVE_CHANNEL_CH2)
          ? isSignalFresh(nowUs, ch2StampUs)
          : isSignalFresh(nowUs, ch3StampUs);
  if (!enableValid) {
    return rcEnableActiveLatched && withinSignalGrace;
  }

  const uint16_t driveUs =
      (rcActiveDriveChannel == DRIVE_CHANNEL_CH2) ? ch2Us : ch3Us;
  const int driveCenterUs = getDriveChannelCenterUs();
  const uint16_t enableUs =
      (rcEnableChannel == DRIVE_CHANNEL_CH2) ? ch2Us : ch3Us;

  if (!allValid) {
    return rcEnableActiveLatched && withinSignalGrace;
  }

  return isRcControlRequested((int)ch1Us, (int)driveUs, driveCenterUs, enableUs);
}

static inline bool isCenteredUs(int pulseUs, int centerUs, int deadbandUs) {
  return abs(pulseUs - centerUs) <= deadbandUs;
}

static inline int pulseUsToScale1000(int pulseUs) {
  const int clampedUs = clampi(pulseUs, RC_PULSE_MIN_US, RC_PULSE_MAX_US);
  return (int)(((long)(clampedUs - RC_PULSE_MIN_US) * 1000L) / (RC_PULSE_MAX_US - RC_PULSE_MIN_US));
}

static int pulseToPositivePwmFromRest(uint16_t pulseUs, int restUs, int deadbandUs, bool highRest) {
  int deltaUs = highRest ? (restUs - (int)pulseUs) : ((int)pulseUs - restUs);
  if (deltaUs <= deadbandUs) {
    return 0;
  }

  deltaUs -= deadbandUs;
  const int spanUs = highRest ? (restUs - RC_PULSE_MIN_US - deadbandUs)
                              : (RC_PULSE_MAX_US - restUs - deadbandUs);
  if (spanUs <= 0) {
    return 0;
  }

  return clampi((int)(((long)deltaUs * PWM_MAX) / spanUs), 0, PWM_MAX);
}

static AxisMode detectAxisMode(int centerUs) {
  if (centerUs < (RC_PULSE_MID_US - 250)) {
    return AXIS_MODE_LOW_REST;
  }
  if (centerUs > (RC_PULSE_MID_US + 250)) {
    return AXIS_MODE_HIGH_REST;
  }
  return AXIS_MODE_CENTERED;
}

static int selectCenteredAxisCenterUs(int measuredUs, int defaultUs, int safetyDeadbandUs) {
  const int limitedUs =
      clampi(measuredUs,
             defaultUs - RC_THROTTLE_CENTER_CAL_WINDOW_US,
             defaultUs + RC_THROTTLE_CENTER_CAL_WINDOW_US);
  return clampi(limitedUs,
                RC_PULSE_MIN_US + safetyDeadbandUs,
                RC_PULSE_MAX_US - safetyDeadbandUs);
}

static int selectSteeringCenterUs(int measuredUs) {
  return selectCenteredAxisCenterUs(measuredUs, RC_STEERING_CENTER_US, RC_STEERING_DRIVE_DEADBAND_US);
}

static int selectThrottleCenterUs(int measuredUs) {
  return selectCenteredAxisCenterUs(measuredUs, RC_THROTTLE_CENTER_US, RC_THROTTLE_DEADBAND_US);
}

static void configureRcNeutralThresholds(int steeringSpanUs, int throttleSpanUs) {
  const int steeringSpanLimitedUs = clampi(steeringSpanUs, 0, 60);
  const int throttleSpanLimitedUs = clampi(throttleSpanUs, 0, 60);

  rcSteeringReadyDeadbandUs =
      maxi(RC_STEERING_READY_DEADBAND_US, (steeringSpanLimitedUs / 2) + 18);
  rcSteeringDriveDeadbandUs =
      maxi(RC_STEERING_DRIVE_DEADBAND_US, (steeringSpanLimitedUs / 2) + 12);
  rcSteeringPivotStartUs =
      maxi(RC_STEERING_PIVOT_START_US, (steeringSpanLimitedUs / 2) + 22);
  rcSteeringPivotKeepUs =
      clampi(maxi(RC_STEERING_PIVOT_KEEP_US, (steeringSpanLimitedUs / 2) + 10),
             RC_STEERING_PIVOT_KEEP_US,
             rcSteeringPivotStartUs - 2);

  rcThrottleNeutralDeadbandUs =
      maxi(RC_THROTTLE_DEADBAND_US, (throttleSpanLimitedUs / 2) + 12);
  rcThrottleIdleExitUs =
      maxi(RC_THROTTLE_IDLE_EXIT_US, rcThrottleNeutralDeadbandUs + 14);
  rcThrottleIdleEnterUs =
      clampi(maxi(RC_THROTTLE_IDLE_ENTER_US, rcThrottleNeutralDeadbandUs - 4),
             RC_THROTTLE_IDLE_ENTER_US,
             rcThrottleIdleExitUs - 4);

  rcThrottleIdleLatched = true;
  rcSteeringPivotLatched = false;
}

static int applyPivotSteeringLatch(int steeringPwm, int pulseUs, int centerUs, uint32_t nowMs) {
  const int deltaUs = abs(pulseUs - centerUs);

  if (steeringPwm == 0) {
    rcSteeringPivotLatched = false;
    rcPivotRequestSinceMs = 0;
    return 0;
  }

  if (rcSteeringPivotLatched) {
    if (deltaUs <= rcSteeringPivotKeepUs) {
      rcSteeringPivotLatched = false;
      rcPivotRequestSinceMs = 0;
      return 0;
    }
    return steeringPwm;
  }

  if (deltaUs <= rcSteeringPivotStartUs) {
    rcPivotRequestSinceMs = 0;
    return 0;
  }

  if (rcPivotRequestSinceMs == 0) {
    rcPivotRequestSinceMs = nowMs;
    return 0;
  }

  if ((nowMs - rcPivotRequestSinceMs) < RC_PIVOT_CONFIRM_MS) {
    return 0;
  }

  rcSteeringPivotLatched = true;
  return steeringPwm;
}

static int pulseToSignedPwm(uint16_t pulseUs, int centerUs, int deadbandUs, bool reverseAxis);

static int getEnableDirection(uint16_t pulseUs, int centerUs) {
  const int deltaUs = (int)pulseUs - centerUs;
  if (abs(deltaUs) < RC_ENABLE_ACTIVE_DELTA_US) {
    return 0;
  }
  return (deltaUs > 0) ? 1 : -1;
}

static const char* getAxisModeName(AxisMode mode) {
  switch (mode) {
    case AXIS_MODE_LOW_REST:
      return "LOW_REST";
    case AXIS_MODE_HIGH_REST:
      return "HIGH_REST";
    default:
      return "CENTERED";
  }
}

static const char* getDriveChannelName(DriveChannel channel) {
  return (channel == DRIVE_CHANNEL_CH2) ? "CH2" : "CH3";
}

static void configureRcChannelRoles(int measuredCh2CenterUs, int measuredCh3CenterUs) {
  rcCh2CenterUs = measuredCh2CenterUs;
  rcCh3CenterUs = selectThrottleCenterUs(measuredCh3CenterUs);
  rcCh2Mode = detectAxisMode(rcCh2CenterUs);
  rcCh3Mode = AXIS_MODE_CENTERED;
  rcActiveDriveChannel = RC_DEFAULT_DRIVE_CHANNEL;
  rcEnableChannel = DRIVE_CHANNEL_CH2;
}

static inline int getDriveChannelCenterUs() {
  return (rcActiveDriveChannel == DRIVE_CHANNEL_CH2) ? rcCh2CenterUs : rcCh3CenterUs;
}

static inline int getFilteredDrivePulseUs() {
  return (rcActiveDriveChannel == DRIVE_CHANNEL_CH2) ? rcFilteredCh2Us : rcFilteredCh3Us;
}

static inline int getFilteredEnablePulseUs() {
  return (rcEnableChannel == DRIVE_CHANNEL_CH2) ? rcFilteredCh2Us : rcFilteredCh3Us;
}

static inline bool isRcEnablePulseActive(uint16_t pulseUs) {
  const int scaled = pulseUsToScale1000((int)pulseUs);

  if (rcEnableActiveLatched) {
    if (scaled < RC_ENABLE_OFF_THRESHOLD_1000) {
      rcEnableActiveLatched = false;
    }
  } else if (scaled > RC_ENABLE_ON_THRESHOLD_1000) {
    rcEnableActiveLatched = true;
  }

  return rcEnableActiveLatched;
}

static inline bool isRcControlRequested(int steeringUs, int driveUs, int driveCenterUs, uint16_t enableUs) {
  (void)steeringUs;
  (void)driveUs;
  (void)driveCenterUs;
  return isRcEnablePulseActive(enableUs);
}

static int pulseToAxisDrivePwm(uint16_t pulseUs,
                               int centerUs,
                               AxisMode mode,
                               bool reverseAxis,
                               int direction) {
  if (mode == AXIS_MODE_CENTERED) {
    return pulseToSignedPwm(pulseUs, centerUs, RC_THROTTLE_DEADBAND_US, reverseAxis);
  }

  const bool highRest = (mode == AXIS_MODE_HIGH_REST);
  const int throttleMag = pulseToPositivePwmFromRest(pulseUs, centerUs, RC_THROTTLE_DEADBAND_US, highRest);
  int throttlePwm = throttleMag * direction;
  if (reverseAxis) {
    throttlePwm = -throttlePwm;
  }
  return throttlePwm;
}

// Sol taraf: FL + RL
// Sag taraf: FR + RR
static inline void driveMotorsPwm(int pwmL, int pwmR) {
  const int pairPwmL = PAIR_DRIVE_MOTOR_SIGN * pwmL;
  const int pairPwmR = PAIR_DRIVE_MOTOR_SIGN * pwmR;
  motorFL.setSpeed((int16_t)(MOTOR_FL_REVERSE ? -pairPwmL : pairPwmL));
  motorRL.setSpeed((int16_t)(MOTOR_RL_REVERSE ? -pairPwmL : pairPwmL));
  motorFR.setSpeed((int16_t)(MOTOR_FR_REVERSE ? -pairPwmR : pairPwmR));
  motorRR.setSpeed((int16_t)(MOTOR_RR_REVERSE ? -pairPwmR : pairPwmR));
}

static inline void driveMotorsWheelPwm(int pwmFL, int pwmRL, int pwmFR, int pwmRR) {
  motorFL.setSpeed((int16_t)(MOTOR_FL_REVERSE ? -pwmFL : pwmFL));
  motorRL.setSpeed((int16_t)(MOTOR_RL_REVERSE ? -pwmRL : pwmRL));
  motorFR.setSpeed((int16_t)(MOTOR_FR_REVERSE ? -pwmFR : pwmFR));
  motorRR.setSpeed((int16_t)(MOTOR_RR_REVERSE ? -pwmRR : pwmRR));
}

static inline void stopMotors() {
  motorFL.stop();
  motorRL.stop();
  motorFR.stop();
  motorRR.stop();
}

static inline void stopDriveCommand() {
  pwmL_cmd = 0;
  pwmR_cmd = 0;
}

static inline void forceSafeStopOutputs() {
  stopDriveCommand();
  resetSerialWheelControl();
  pwmL_out = 0;
  pwmR_out = 0;
  rcDebugThrottlePwm = 0;
  rcDebugSteeringPwm = 0;
  rcDebugPwmLCmd = 0;
  rcDebugPwmRCmd = 0;
  outputActiveStopPwmStepIntervalMs = OUTPUT_DRIVE_STOP_PWM_STEP_INTERVAL_MS;
  pwmLReverseHoldUntilMs = 0;
  pwmRReverseHoldUntilMs = 0;
  pwmLReverseWaitStartMs = 0;
  pwmRReverseWaitStartMs = 0;
  pwmLReverseNeutralUntilMs = 0;
  pwmRReverseNeutralUntilMs = 0;
  pwmLZeroStopCarryMs = 0;
  pwmRZeroStopCarryMs = 0;
  pwmLReverseRestartActive = false;
  pwmRReverseRestartActive = false;
  wheelBalanceLeftPairTrim = 0;
  wheelBalanceRightPairTrim = 0;
  wheelBalanceSideTrim = 0;
  stopMotors();
}

static inline void resetRcPivotState() {
  rcPivotActive = false;
  rcPivotRequestSinceMs = 0;
  rcSteeringPivotLatched = false;
}

static inline int signi(int value) {
  if (value > 0) return 1;
  if (value < 0) return -1;
  return 0;
}

static inline bool isForwardReverseMotion(int pwmL, int pwmR) {
  const int signL = signi(pwmL);
  const int signR = signi(pwmR);
  if (signL == 0 && signR == 0) {
    return false;
  }
  return (signL == signR) || (signL == 0) || (signR == 0);
}

static inline void clampForwardReversePwm(int& pwmL, int& pwmR) {
  if (!isForwardReverseMotion(pwmL, pwmR)) {
    return;
  }
  int maxMag = abs(pwmL);
  if (abs(pwmR) > maxMag) {
    maxMag = abs(pwmR);
  }
  if (maxMag <= DRIVE_MAX_PWM) {
    return;
  }
  pwmL = (pwmL * DRIVE_MAX_PWM) / maxMag;
  pwmR = (pwmR * DRIVE_MAX_PWM) / maxMag;
}

static inline int clampSignedMagnitude(int value, int maxMagnitude) {
  const int limitedMagnitude = abs(maxMagnitude);
  if (value > limitedMagnitude) {
    return limitedMagnitude;
  }
  if (value < -limitedMagnitude) {
    return -limitedMagnitude;
  }
  return value;
}

static inline void clampForwardReverseWheelPwm(int& pwmFL,
                                               int& pwmRL,
                                               int& pwmFR,
                                               int& pwmRR) {
  int maxMag = abs(pwmFL);
  if (abs(pwmRL) > maxMag) maxMag = abs(pwmRL);
  if (abs(pwmFR) > maxMag) maxMag = abs(pwmFR);
  if (abs(pwmRR) > maxMag) maxMag = abs(pwmRR);
  if (maxMag <= DRIVE_MAX_PWM) {
    return;
  }
  pwmFL = (pwmFL * DRIVE_MAX_PWM) / maxMag;
  pwmRL = (pwmRL * DRIVE_MAX_PWM) / maxMag;
  pwmFR = (pwmFR * DRIVE_MAX_PWM) / maxMag;
  pwmRR = (pwmRR * DRIVE_MAX_PWM) / maxMag;
}

static inline int addMagnitudeTrim(int base, int trim) {
  const int s = signi(base);
  if (s == 0 || trim == 0) {
    return base;
  }
  return clampi(base + (s * trim), -PWM_MAX, PWM_MAX);
}

static float getSideMaxAbsRpm(bool leftSide) {
  if (!ENABLE_ENCODER_MONITORING) {
    return 0.0f;
  }

  if (leftSide) {
    const float flAbsRpm = absf_local(encFL.getRPM());
    const float rlAbsRpm = absf_local(encRL.getRPM());
    return (flAbsRpm > rlAbsRpm) ? flAbsRpm : rlAbsRpm;
  }

  const float frAbsRpm = absf_local(encFR.getRPM());
  const float rrAbsRpm = RR_ENCODER_ENABLED ? absf_local(encRR.getRPM()) : frAbsRpm;
  return (frAbsRpm > rrAbsRpm) ? frAbsRpm : rrAbsRpm;
}

static void computeBalancedWheelPwm(int baseL,
                                    int baseR,
                                    uint32_t deltaMs,
                                    int& outFL,
                                    int& outRL,
                                    int& outFR,
                                    int& outRR) {
  outFL = baseL;
  outRL = baseL;
  outFR = baseR;
  outRR = baseR;

  const int trimSlewStep = clampi((int)((unsigned long)WHEEL_BALANCE_TRIM_SLEW_PWM_PER_MS * deltaMs), 1, PWM_MAX);

  int desiredLeftPairTrim = 0;
  int desiredRightPairTrim = 0;
  int desiredSideTrim = 0;

  const bool balanceAllowed =
      ENABLE_WHEEL_SPEED_BALANCE &&
      ENABLE_ENCODER_MONITORING &&
      (abs(baseL) >= WHEEL_BALANCE_MIN_ACTIVE_PWM || abs(baseR) >= WHEEL_BALANCE_MIN_ACTIVE_PWM);

  if (balanceAllowed) {
    const float flAbsRpm = absf_local(encFL.getRPM());
    const float frAbsRpm = absf_local(encFR.getRPM());
    const float rlAbsRpm = absf_local(encRL.getRPM());
    const float rrAbsRpm = RR_ENCODER_ENABLED ? absf_local(encRR.getRPM()) : frAbsRpm;

    if (abs(baseL) >= WHEEL_BALANCE_MIN_ACTIVE_PWM &&
        (flAbsRpm >= WHEEL_BALANCE_MIN_ACTIVE_RPM || rlAbsRpm >= WHEEL_BALANCE_MIN_ACTIVE_RPM)) {
      const float leftPairErr = applyDeadbandFloat(flAbsRpm - rlAbsRpm, WHEEL_BALANCE_PAIR_DEADBAND_RPM);
      desiredLeftPairTrim = clampi((int)(leftPairErr * WHEEL_BALANCE_PAIR_KP),
                                   -WHEEL_BALANCE_PAIR_MAX_TRIM_PWM,
                                   WHEEL_BALANCE_PAIR_MAX_TRIM_PWM);
    }

    if (abs(baseR) >= WHEEL_BALANCE_MIN_ACTIVE_PWM &&
        (frAbsRpm >= WHEEL_BALANCE_MIN_ACTIVE_RPM || rrAbsRpm >= WHEEL_BALANCE_MIN_ACTIVE_RPM)) {
      const float rightPairErr = applyDeadbandFloat(frAbsRpm - rrAbsRpm, WHEEL_BALANCE_PAIR_DEADBAND_RPM);
      desiredRightPairTrim = clampi((int)(rightPairErr * WHEEL_BALANCE_PAIR_KP),
                                    -WHEEL_BALANCE_PAIR_MAX_TRIM_PWM,
                                    WHEEL_BALANCE_PAIR_MAX_TRIM_PWM);
    }

    const bool nearStraightCommand =
        signi(baseL) == signi(baseR) &&
        abs(abs(baseL) - abs(baseR)) <= RC_DRIVE_SELECT_MARGIN_PWM;

    if (nearStraightCommand &&
        abs(baseL) >= WHEEL_BALANCE_MIN_ACTIVE_PWM &&
        abs(baseR) >= WHEEL_BALANCE_MIN_ACTIVE_PWM) {
      const float leftAvgAbsRpm = 0.5f * (flAbsRpm + rlAbsRpm);
      const float rightAvgAbsRpm = 0.5f * (frAbsRpm + rrAbsRpm);

      if (leftAvgAbsRpm >= WHEEL_BALANCE_MIN_ACTIVE_RPM ||
          rightAvgAbsRpm >= WHEEL_BALANCE_MIN_ACTIVE_RPM) {
        const float leftResponse = leftAvgAbsRpm / (float)abs(baseL);
        const float rightResponse = rightAvgAbsRpm / (float)abs(baseR);
        const float sideErr = applyDeadbandFloat(leftResponse - rightResponse,
                                                 WHEEL_BALANCE_SIDE_DEADBAND_RATIO);
        desiredSideTrim = clampi((int)(sideErr * WHEEL_BALANCE_SIDE_KP),
                                 -WHEEL_BALANCE_SIDE_MAX_TRIM_PWM,
                                 WHEEL_BALANCE_SIDE_MAX_TRIM_PWM);
      }
    }
  }

  wheelBalanceLeftPairTrim = rampToward(wheelBalanceLeftPairTrim, desiredLeftPairTrim, trimSlewStep);
  wheelBalanceRightPairTrim = rampToward(wheelBalanceRightPairTrim, desiredRightPairTrim, trimSlewStep);
  wheelBalanceSideTrim = rampToward(wheelBalanceSideTrim, desiredSideTrim, trimSlewStep);

  outFL = addMagnitudeTrim(outFL, -wheelBalanceLeftPairTrim);
  outRL = addMagnitudeTrim(outRL, +wheelBalanceLeftPairTrim);
  outFR = addMagnitudeTrim(outFR, -wheelBalanceRightPairTrim);
  outRR = addMagnitudeTrim(outRR, +wheelBalanceRightPairTrim);

  outFL = addMagnitudeTrim(outFL, -wheelBalanceSideTrim);
  outRL = addMagnitudeTrim(outRL, -wheelBalanceSideTrim);
  outFR = addMagnitudeTrim(outFR, +wheelBalanceSideTrim);
  outRR = addMagnitudeTrim(outRR, +wheelBalanceSideTrim);
}

static const char* classifyUserSerialDirection(int pwmL, int pwmR) {
  if (pwmL == 0 && pwmR == 0) {
    return "STOP";
  }
  if (pwmL > 0 && pwmR > 0) {
    return "FWD";
  }
  if (pwmL < 0 && pwmR < 0) {
    return "REV";
  }
  if (pwmL > 0 && pwmR < 0) {
    return "TURN_RIGHT";
  }
  if (pwmL < 0 && pwmR > 0) {
    return "TURN_LEFT";
  }
  return "MIXED";
}

static int rampToward(int current, int target, int maxStep) {
  if (current < target) {
    current += maxStep;
    if (current > target) current = target;
  } else if (current > target) {
    current -= maxStep;
    if (current < target) current = target;
  }
  return current;
}

static inline void updateRawEncoderChannel(uint8_t pinA,
                                           uint8_t pinB,
                                           uint8_t& lastA,
                                           uint8_t& lastB,
                                           uint32_t& edgeCount) {
  const uint8_t newA = (uint8_t)digitalRead(pinA);
  const uint8_t newB = (uint8_t)digitalRead(pinB);
  if (newA != lastA || newB != lastB) {
    edgeCount++;
    lastA = newA;
    lastB = newB;
  }
}

static void applySerialWheelTestOutputs() {
  if (!serialWheelModeActive ||
      serialWheelId == SERIAL_WHEEL_NONE ||
      serialWheelTargetRpmCmd == 0) {
    resetSerialWheelControl();
    pwmL_out = 0;
    pwmR_out = 0;
    stopMotors();
    return;
  }

  const uint32_t nowMs = millis();
  uint32_t deltaMs = nowMs - serialWheelLastControlMs;
  if (serialWheelLastControlMs == 0 || deltaMs > 100U) {
    deltaMs = 20U;
  }
  serialWheelLastControlMs = nowMs;

  const float targetRpm = (float)serialWheelTargetRpmCmd;
  serialWheelMeasuredRpm = getSerialWheelMeasuredRpm(serialWheelId);
  const float errorRpm = targetRpm - serialWheelMeasuredRpm;
  const float deltaSec = 0.001f * (float)deltaMs;

  const float proposedIntegral =
      clampf_local(serialWheelIntegral + (errorRpm * deltaSec),
                   -SERIAL_WHEEL_TEST_INTEGRAL_LIMIT,
                   SERIAL_WHEEL_TEST_INTEGRAL_LIMIT);
  const float feedforwardPwm = targetRpm * SERIAL_WHEEL_TEST_FEEDFORWARD_PWM_PER_RPM;
  float rawPwm = feedforwardPwm +
                 (errorRpm * SERIAL_WHEEL_TEST_KP) +
                 (proposedIntegral * SERIAL_WHEEL_TEST_KI);

  const bool saturatedHigh = rawPwm > (float)SERIAL_WHEEL_TEST_MAX_PWM;
  const bool saturatedLow = rawPwm < (float)(-SERIAL_WHEEL_TEST_MAX_PWM);
  if (!((saturatedHigh && errorRpm > 0.0f) ||
        (saturatedLow && errorRpm < 0.0f))) {
    serialWheelIntegral = proposedIntegral;
    rawPwm = feedforwardPwm +
             (errorRpm * SERIAL_WHEEL_TEST_KP) +
             (serialWheelIntegral * SERIAL_WHEEL_TEST_KI);
  }

  int targetPwm =
      (int)((rawPwm >= 0.0f) ? (rawPwm + 0.5f) : (rawPwm - 0.5f));
  targetPwm = clampi(targetPwm,
                     -SERIAL_WHEEL_TEST_MAX_PWM,
                     SERIAL_WHEEL_TEST_MAX_PWM);

  if (targetPwm != 0 && abs(targetPwm) < SERIAL_WHEEL_TEST_MIN_ACTIVE_PWM) {
    targetPwm = signi(targetPwm) * SERIAL_WHEEL_TEST_MIN_ACTIVE_PWM;
  }

  const int slewStep =
      clampi((int)((unsigned long)SERIAL_WHEEL_TEST_PWM_SLEW_PER_MS * deltaMs),
             1,
             PWM_MAX);
  serialWheelPwmCmd = rampToward(serialWheelPwmCmd, targetPwm, slewStep);
  serialWheelPwmOut = serialWheelPwmCmd;

  int pwmFL = 0;
  int pwmFR = 0;
  int pwmRL = 0;
  int pwmRR = 0;

  switch (serialWheelId) {
    case SERIAL_WHEEL_FL:
      pwmFL = serialWheelPwmOut;
      break;
    case SERIAL_WHEEL_FR:
      pwmFR = serialWheelPwmOut;
      break;
    case SERIAL_WHEEL_RL:
      pwmRL = serialWheelPwmOut;
      break;
    case SERIAL_WHEEL_RR:
      pwmRR = serialWheelPwmOut;
      break;
    default:
      break;
  }

  pwmL_out = 0;
  pwmR_out = 0;
  driveMotorsWheelPwm(pwmFL, pwmRL, pwmFR, pwmRR);
}

static void beginRawEncoderMonitor() {
  rawFlA = (uint8_t)digitalRead(PIN_ENCODER_FL_A);
  rawFlB = (uint8_t)digitalRead(PIN_ENCODER_FL_B);
  rawFrA = (uint8_t)digitalRead(PIN_ENCODER_FR_A);
  rawFrB = (uint8_t)digitalRead(PIN_ENCODER_FR_B);
  rawRlA = (uint8_t)digitalRead(PIN_ENCODER_RL_A);
  rawRlB = (uint8_t)digitalRead(PIN_ENCODER_RL_B);
  rawRrA = RR_ENCODER_ENABLED ? (uint8_t)digitalRead(PIN_ENCODER_RR_A) : 0;
  rawRrB = RR_ENCODER_ENABLED ? (uint8_t)digitalRead(PIN_ENCODER_RR_B) : 0;
  rawFlEdges = 0;
  rawFrEdges = 0;
  rawRlEdges = 0;
  rawRrEdges = 0;
}

static void updateRawEncoderMonitor() {
  updateRawEncoderChannel(PIN_ENCODER_FL_A, PIN_ENCODER_FL_B, rawFlA, rawFlB, rawFlEdges);
  updateRawEncoderChannel(PIN_ENCODER_FR_A, PIN_ENCODER_FR_B, rawFrA, rawFrB, rawFrEdges);
  updateRawEncoderChannel(PIN_ENCODER_RL_A, PIN_ENCODER_RL_B, rawRlA, rawRlB, rawRlEdges);
  if (RR_ENCODER_ENABLED) {
    updateRawEncoderChannel(PIN_ENCODER_RR_A, PIN_ENCODER_RR_B, rawRrA, rawRrB, rawRrEdges);
  } else {
    rawRrA = 0;
    rawRrB = 0;
    rawRrEdges = 0;
  }
}

static void applyDriveOutputs() {
  const uint32_t nowMs = millis();
  uint32_t deltaMs = nowMs - lastOutputUpdateMs;
  if (lastOutputUpdateMs == 0 || deltaMs > 100U) {
    deltaMs = 1U;
  }
  lastOutputUpdateMs = nowMs;

  if (!OUTPUT_SOFT_STOP_ENABLED) {
    pwmL_out = pwmL_cmd;
    pwmR_out = pwmR_cmd;
  } else {
    const int accelStep = clampi((int)((unsigned long)OUTPUT_ACCEL_PWM_PER_MS * deltaMs), 1, PWM_MAX);
    const int decelStep = clampi((int)((unsigned long)OUTPUT_DECEL_PWM_PER_MS * deltaMs), 1, PWM_MAX);
    const int reverseDecelStep = clampi((int)((unsigned long)OUTPUT_REVERSE_DECEL_PWM_PER_MS * deltaMs), 1, PWM_MAX);
    const int reverseAccelStep = clampi((int)((unsigned long)OUTPUT_REVERSE_ACCEL_PWM_PER_MS * deltaMs), 1, PWM_MAX);
    const int emergencyStep = clampi((int)((unsigned long)OUTPUT_EMERGENCY_STOP_PWM_PER_MS * deltaMs), 1, PWM_MAX);
    const bool emergencyStop = (rcSignalLostSinceMs != 0);
    const uint32_t zeroStopIntervalMs = (outputActiveStopPwmStepIntervalMs == 0)
                                            ? 1
                                            : outputActiveStopPwmStepIntervalMs;
    const float leftAbsRpm = getSideMaxAbsRpm(true);
    const float rightAbsRpm = getSideMaxAbsRpm(false);

    auto rampOutput = [&](int current,
                          int target,
                          uint32_t& reverseWaitStartMs,
                          uint32_t& zeroStopCarryMs,
                          bool& reverseRestartActive,
                          float sideAbsRpm) -> int {
      if (emergencyStop) {
        reverseWaitStartMs = 0;
        zeroStopCarryMs = 0;
        reverseRestartActive = false;
        return rampToward(current, 0, emergencyStep);
      }

      if (target == current && target != 0) {
        zeroStopCarryMs = 0;
        return current;
      }

      if (target == 0) {
        reverseWaitStartMs = 0;
        reverseRestartActive = false;
        if (current == 0) {
          zeroStopCarryMs = 0;
          return 0;
        }

        zeroStopCarryMs += deltaMs;
        const uint32_t zeroStopSteps = zeroStopCarryMs / zeroStopIntervalMs;
        if (zeroStopSteps == 0) {
          return current;
        }

        zeroStopCarryMs -= zeroStopSteps * zeroStopIntervalMs;
        return rampToward(current, 0, clampi((int)zeroStopSteps, 1, PWM_MAX));
      }

      zeroStopCarryMs = 0;

      if (current != 0 && signi(current) != signi(target)) {
        if (reverseWaitStartMs == 0) {
          reverseWaitStartMs = nowMs;
        }
        reverseRestartActive = true;
        return rampToward(current, 0, reverseDecelStep);
      }

      if (current == 0 && reverseRestartActive) {
        const bool rpmReleased = (!ENABLE_ENCODER_MONITORING) || (sideAbsRpm <= OUTPUT_REVERSE_RELEASE_RPM);
        const bool reverseTimedOut =
            reverseWaitStartMs != 0 && ((nowMs - reverseWaitStartMs) >= OUTPUT_REVERSE_MAX_WAIT_MS);

        if (!rpmReleased && !reverseTimedOut) {
          return 0;
        }

        if (target != 0) {
          const int restarted = rampToward(current, target, reverseAccelStep);
          if (restarted == target) {
            reverseWaitStartMs = 0;
            reverseRestartActive = false;
          }
          return restarted;
        }
      }

      reverseWaitStartMs = 0;
      reverseRestartActive = false;

      const int step = (abs(target) > abs(current)) ? accelStep : decelStep;
      return rampToward(current, target, step);
    };

    pwmL_out = rampOutput(pwmL_out, pwmL_cmd, pwmLReverseWaitStartMs, pwmLZeroStopCarryMs, pwmLReverseRestartActive, leftAbsRpm);
    pwmR_out = rampOutput(pwmR_out, pwmR_cmd, pwmRReverseWaitStartMs, pwmRZeroStopCarryMs, pwmRReverseRestartActive, rightAbsRpm);
  }

  clampForwardReversePwm(pwmL_out, pwmR_out);

  if (pwmL_out == 0 && pwmR_out == 0) {
    outputActiveStopPwmStepIntervalMs = OUTPUT_DRIVE_STOP_PWM_STEP_INTERVAL_MS;
    stopMotors();
  } else {
    driveMotorsPwm(pwmL_out, pwmR_out);
  }
}

static void mixRcToDrive(int throttlePwm, int steeringPwm, int& outL, int& outR) {
  throttlePwm = clampSignedMagnitude(throttlePwm, DRIVE_MAX_PWM);

  if (throttlePwm == 0) {
    const int pivot = clampSignedMagnitude(steeringPwm, RC_PIVOT_MAX_PWM);
    outL = pivot;
    outR = -pivot;
    return;
  }

  const int throttleAbs = abs(throttlePwm);
  const int throttleSign = signi(throttlePwm);
  const int steeringAbs = clampi(abs(steeringPwm), 0, PWM_MAX);
  const int maxReductionPct = 100 - RC_DRIVE_MIN_INNER_THROTTLE_PCT;
  const int reductionPct = (steeringAbs * maxReductionPct) / PWM_MAX;
  const int outerBoostAbs = (throttleAbs * steeringAbs * RC_DRIVE_OUTER_BOOST_PCT) / (PWM_MAX * 100);
  const int outerAbs = clampi(throttleAbs + outerBoostAbs, 0, DRIVE_MAX_PWM);
  int innerScalePct = 100 - ((reductionPct * RC_DRIVE_INNER_AGGRESSION_PCT) / 100);
  const bool allowCounterThrottle =
      throttleAbs <= RC_DRIVE_COUNTER_THROTTLE_MAX_PWM &&
      steeringAbs >= ((PWM_MAX * RC_DRIVE_COUNTER_STEER_MIN_PCT) / 100);
  if (!allowCounterThrottle && innerScalePct < 0) {
    innerScalePct = 0;
  }
  const int innerSignedMag = (throttleAbs * innerScalePct) / 100;
  const int outerCmd = throttleSign * outerAbs;
  const int innerCmd = clampSignedMagnitude(throttleSign * innerSignedMag, DRIVE_MAX_PWM);

  if (steeringPwm > 0) {
    outL = innerCmd;
    outR = outerCmd;
    return;
  }

  if (steeringPwm < 0) {
    outL = outerCmd;
    outR = innerCmd;
    return;
  }

  outL = outerCmd;
  outR = outerCmd;
}

static int pulseToSignedPwm(uint16_t pulseUs, int centerUs, int deadbandUs, bool reverseAxis) {
  int centered = (int)pulseUs - centerUs;
  centered = applyDeadband(centered, deadbandUs);

  const int positiveSpan = RC_PULSE_MAX_US - centerUs - deadbandUs;
  const int negativeSpan = centerUs - RC_PULSE_MIN_US - deadbandUs;

  int pwm = 0;
  if (centered > 0) {
    pwm = ((long)centered * PWM_MAX) / positiveSpan;
  } else if (centered < 0) {
    pwm = ((long)centered * PWM_MAX) / negativeSpan;
  }

  pwm = clampi(pwm, -PWM_MAX, PWM_MAX);
  return reverseAxis ? -pwm : pwm;
}

static int clampThrottleToIdle(int pwm, int pulseUs, int centerUs) {
  const int deltaUs = abs(pulseUs - centerUs);

  if (rcThrottleIdleLatched) {
    if (deltaUs <= rcThrottleIdleExitUs) {
      return 0;
    }
    rcThrottleIdleLatched = false;
    return pwm;
  }

  if (deltaUs <= rcThrottleIdleEnterUs) {
    rcThrottleIdleLatched = true;
    return 0;
  }

  return pwm;
}

static inline bool isSignalFresh(uint32_t nowUs, uint32_t lastPulseUs) {
  return lastPulseUs != 0 && (nowUs - lastPulseUs) <= RC_SIGNAL_TIMEOUT_US;
}

static bool sampleRcPulseDirect(uint8_t pin, uint16_t& outPulseUs) {
  const unsigned long widthUs = pulseIn(pin, HIGH, RC_DIRECT_PULSE_TIMEOUT_US);
  if (widthUs < 900UL || widthUs > 2100UL) {
    return false;
  }
  outPulseUs = (uint16_t)widthUs;
  return true;
}

static bool refreshRcSignalsDirect(bool forceSampleAll) {
  const uint32_t nowMs = millis();
  if (!forceSampleAll &&
      rcLastDirectSampleMs != 0 &&
      (nowMs - rcLastDirectSampleMs) < RC_DIRECT_SAMPLE_RETRY_MS) {
    return false;
  }
  rcLastDirectSampleMs = nowMs;

  const uint32_t snapshotUs = micros();
  bool updatedAny = false;

  struct ChannelDirectSample {
    uint8_t pin;
    volatile uint16_t* pulseUs;
    volatile uint32_t* lastPulseUs;
  };

  ChannelDirectSample channels[] = {
      {CH1_PIN, &ch1PulseUs, &ch1LastPulseUs},
      {CH2_PIN, &ch2PulseUs, &ch2LastPulseUs},
      {CH3_PIN, &ch3PulseUs, &ch3LastPulseUs},
  };

  for (size_t i = 0; i < (sizeof(channels) / sizeof(channels[0])); ++i) {
    if (!forceSampleAll && isSignalFresh(snapshotUs, *channels[i].lastPulseUs)) {
      continue;
    }

    uint16_t directPulseUs = 0;
    if (!sampleRcPulseDirect(channels[i].pin, directPulseUs)) {
      continue;
    }

    const uint32_t stampUs = micros();
    noInterrupts();
    *channels[i].pulseUs = directPulseUs;
    *channels[i].lastPulseUs = stampUs;
    interrupts();
    updatedAny = true;
  }

  return updatedAny;
}

static void readRcSnapshot(uint16_t& ch1Us,
                           uint16_t& ch2Us,
                           uint16_t& ch3Us,
                           uint32_t& ch1StampUs,
                           uint32_t& ch2StampUs,
                           uint32_t& ch3StampUs) {
  noInterrupts();
  ch1Us = ch1PulseUs;
  ch2Us = ch2PulseUs;
  ch3Us = ch3PulseUs;
  ch1StampUs = ch1LastPulseUs;
  ch2StampUs = ch2LastPulseUs;
  ch3StampUs = ch3LastPulseUs;
  interrupts();
}

static void calibrateRcCenters() {
  uint32_t steeringSum = 0;
  uint32_t throttleSum = 0;
  uint32_t enableSum = 0;
  uint16_t sampleCount = 0;
  uint16_t ch1Min = 65535;
  uint16_t ch1Max = 0;
  uint16_t ch2Min = 65535;
  uint16_t ch2Max = 0;
  uint16_t ch3Min = 65535;
  uint16_t ch3Max = 0;
  const uint32_t startMs = millis();

  while ((millis() - startMs) < RC_CALIBRATION_MS) {
    refreshRcSignalsDirect(false);

    uint16_t ch1Us = 0;
    uint16_t ch2Us = 0;
    uint16_t ch3Us = 0;
    uint32_t ch1StampUs = 0;
    uint32_t ch2StampUs = 0;
    uint32_t ch3StampUs = 0;
    const uint32_t nowUs = micros();

    readRcSnapshot(ch1Us, ch2Us, ch3Us, ch1StampUs, ch2StampUs, ch3StampUs);

    if (isSignalFresh(nowUs, ch1StampUs) &&
        isSignalFresh(nowUs, ch2StampUs) &&
        isSignalFresh(nowUs, ch3StampUs)) {
      steeringSum += ch1Us;
      throttleSum += ch2Us;
      enableSum += ch3Us;
      if (ch1Us < ch1Min) ch1Min = ch1Us;
      if (ch1Us > ch1Max) ch1Max = ch1Us;
      if (ch2Us < ch2Min) ch2Min = ch2Us;
      if (ch2Us > ch2Max) ch2Max = ch2Us;
      if (ch3Us < ch3Min) ch3Min = ch3Us;
      if (ch3Us > ch3Max) ch3Max = ch3Us;
      sampleCount++;
    }

    delay(5);
  }

  const bool stableCapture =
      sampleCount >= RC_MIN_CALIBRATION_SAMPLES &&
      (int)(ch1Max - ch1Min) <= RC_CALIBRATION_STABLE_SPAN_US &&
      (int)(ch2Max - ch2Min) <= RC_CALIBRATION_STABLE_SPAN_US &&
      (int)(ch3Max - ch3Min) <= RC_CALIBRATION_STABLE_SPAN_US;
  const bool haveEnoughSamples = sampleCount >= RC_MIN_CALIBRATION_SAMPLES;

  if (haveEnoughSamples) {
    const int measuredCh2CenterUs = (int)(throttleSum / sampleCount);
    const int measuredCh3CenterUs = (int)(enableSum / sampleCount);
    rcSteeringCenterUs = selectSteeringCenterUs((int)(steeringSum / sampleCount));
    configureRcChannelRoles(measuredCh2CenterUs, measuredCh3CenterUs);
    configureRcNeutralThresholds((int)(ch1Max - ch1Min), (int)(ch3Max - ch3Min));
    rcFilteredCh1Us = rcSteeringCenterUs;
    rcFilteredCh2Us = rcCh2CenterUs;
    rcFilteredCh3Us = rcCh3CenterUs;
    resetRcPivotState();
    rcLastFilterUpdateUs = micros();

    rcNeutralReady = false;
    rcNeutralSinceMs = 0;
    rcEnableReleasedSinceReady = false;
    rcControlArmed = false;

    if (stableCapture) {
      Serial.print("RC_CALIBRATED ");
    } else {
      Serial.print("RC_CALIBRATED_RELAXED ");
    }
    Serial.print(rcSteeringCenterUs); Serial.print(' ');
    Serial.print(rcCh2CenterUs); Serial.print(' ');
    Serial.print(rcCh3CenterUs); Serial.print(' ');
    Serial.println(sampleCount);
    Serial.print("RC_MEASURED_CH3 ");
    Serial.println(measuredCh3CenterUs);
    Serial.println("RC_WAIT_NEUTRAL");
    Serial.print("RC_CHANNEL_MODES CH2=");
    Serial.print(getAxisModeName(rcCh2Mode));
    Serial.print(" CH3=");
    Serial.println(getAxisModeName(rcCh3Mode));
    Serial.print("RC_CHANNEL_MAP STEERING=CH1 THROTTLE=");
    Serial.print(getDriveChannelName(rcActiveDriveChannel));
    Serial.print(" ENABLE=");
    Serial.println(getDriveChannelName(rcEnableChannel));
    Serial.print("RC_NEUTRAL_WINDOWS STR_READY=");
    Serial.print(rcSteeringReadyDeadbandUs);
    Serial.print(" STR_DRIVE=");
    Serial.print(rcSteeringDriveDeadbandUs);
    Serial.print(" PIVOT_START=");
    Serial.print(rcSteeringPivotStartUs);
    Serial.print(" PIVOT_KEEP=");
    Serial.print(rcSteeringPivotKeepUs);
    Serial.print(" THR_DB=");
    Serial.print(rcThrottleNeutralDeadbandUs);
    Serial.print(" THR_EXIT=");
    Serial.print(rcThrottleIdleExitUs);
    Serial.print(" THR_ENTER=");
    Serial.println(rcThrottleIdleEnterUs);
  } else {
    uint16_t ch1Us = 0;
    uint16_t ch2Us = 0;
    uint16_t ch3Us = 0;
    uint32_t ch1StampUs = 0;
    uint32_t ch2StampUs = 0;
    uint32_t ch3StampUs = 0;
    const uint32_t nowUs = micros();

    readRcSnapshot(ch1Us, ch2Us, ch3Us, ch1StampUs, ch2StampUs, ch3StampUs);

    if (isSignalFresh(nowUs, ch1StampUs) &&
        isSignalFresh(nowUs, ch2StampUs) &&
        isSignalFresh(nowUs, ch3StampUs)) {
      rcSteeringCenterUs = selectSteeringCenterUs(ch1Us);
      configureRcChannelRoles(ch2Us, ch3Us);
      configureRcNeutralThresholds(0, 0);
      rcFilteredCh1Us = rcSteeringCenterUs;
      rcFilteredCh2Us = rcCh2CenterUs;
      rcFilteredCh3Us = rcCh3CenterUs;
      resetRcPivotState();
      rcLastFilterUpdateUs = micros();

      rcNeutralReady = false;
      rcNeutralSinceMs = 0;
      rcEnableReleasedSinceReady = false;
      rcControlArmed = false;

      Serial.print("RC_CALIBRATED_FALLBACK ");
      Serial.print(rcSteeringCenterUs); Serial.print(' ');
      Serial.print(rcCh2CenterUs); Serial.print(' ');
      Serial.print(rcCh3CenterUs);
      Serial.println();
      Serial.print("RC_MEASURED_CH3 ");
      Serial.println(ch3Us);
      Serial.println("RC_WAIT_NEUTRAL");
      Serial.print("RC_CHANNEL_MODES CH2=");
      Serial.print(getAxisModeName(rcCh2Mode));
      Serial.print(" CH3=");
      Serial.println(getAxisModeName(rcCh3Mode));
      Serial.print("RC_CHANNEL_MAP STEERING=CH1 THROTTLE=");
      Serial.print(getDriveChannelName(rcActiveDriveChannel));
      Serial.print(" ENABLE=");
      Serial.println(getDriveChannelName(rcEnableChannel));
      Serial.print("RC_NEUTRAL_WINDOWS STR_READY=");
      Serial.print(rcSteeringReadyDeadbandUs);
      Serial.print(" STR_DRIVE=");
      Serial.print(rcSteeringDriveDeadbandUs);
      Serial.print(" PIVOT_START=");
      Serial.print(rcSteeringPivotStartUs);
      Serial.print(" PIVOT_KEEP=");
      Serial.print(rcSteeringPivotKeepUs);
      Serial.print(" THR_DB=");
      Serial.print(rcThrottleNeutralDeadbandUs);
      Serial.print(" THR_EXIT=");
      Serial.print(rcThrottleIdleExitUs);
      Serial.print(" THR_ENTER=");
      Serial.println(rcThrottleIdleEnterUs);
    } else {
      rcNeutralReady = false;
      rcNeutralSinceMs = 0;
      rcEnableReleasedSinceReady = false;
      rcControlArmed = false;
      configureRcChannelRoles(RC_THROTTLE_CENTER_US, RC_ENABLE_CENTER_US);
      configureRcNeutralThresholds(0, 0);
      rcFilteredCh1Us = RC_STEERING_CENTER_US;
      rcFilteredCh2Us = RC_THROTTLE_CENTER_US;
      rcFilteredCh3Us = RC_PULSE_MID_US;
      resetRcPivotState();
      rcLastFilterUpdateUs = micros();
      Serial.println("RC_CALIBRATION_FAILED");
    }
  }

  Serial.print("RC_CENTER ");
  Serial.print(rcSteeringCenterUs); Serial.print(' ');
  Serial.print(rcCh2CenterUs); Serial.print(' ');
  Serial.println(rcCh3CenterUs);
}

// ===============================
// Non-blocking RC pulse capture
// ===============================
static inline void captureRcPulse(volatile uint32_t& riseUs,
                                  volatile uint16_t& pulseUs,
                                  volatile uint32_t& lastPulseUs,
                                  uint8_t pin) {
  uint32_t nowUs = micros();

  if (digitalRead(pin) == HIGH) {
    riseUs = nowUs;
    return;
  }

  uint32_t widthUs = nowUs - riseUs;
  if (widthUs >= 900 && widthUs <= 2100) {
    pulseUs = (uint16_t)widthUs;
    lastPulseUs = nowUs;
  }
}

void isrCh1() {
  captureRcPulse(ch1RiseUs, ch1PulseUs, ch1LastPulseUs, CH1_PIN);
}

void isrCh2() {
  captureRcPulse(ch2RiseUs, ch2PulseUs, ch2LastPulseUs, CH2_PIN);
}

void isrCh3() {
  captureRcPulse(ch3RiseUs, ch3PulseUs, ch3LastPulseUs, CH3_PIN);
}

static void updateRcCommand() {
  refreshRcSignalsDirect(false);

  uint16_t ch1Us = 0;
  uint16_t ch2Us = 0;
  uint16_t ch3Us = 0;
  uint32_t ch1StampUs = 0;
  uint32_t ch2StampUs = 0;
  uint32_t ch3StampUs = 0;

  readRcSnapshot(ch1Us, ch2Us, ch3Us, ch1StampUs, ch2StampUs, ch3StampUs);

  const uint32_t nowUs = micros();
  const bool ch1Valid = isSignalFresh(nowUs, ch1StampUs);
  const bool ch2Valid = isSignalFresh(nowUs, ch2StampUs);
  const bool ch3Valid = isSignalFresh(nowUs, ch3StampUs);
  uint32_t filterDeltaUs = nowUs - rcLastFilterUpdateUs;
  if (rcLastFilterUpdateUs == 0 || filterDeltaUs > 100000UL) {
    filterDeltaUs = 1000UL;
  }
  rcLastFilterUpdateUs = nowUs;

  if (!(ch1Valid && ch2Valid && ch3Valid)) {
    refreshRcSignalsDirect(true);
    readRcSnapshot(ch1Us, ch2Us, ch3Us, ch1StampUs, ch2StampUs, ch3StampUs);

    const uint32_t retryNowUs = micros();
    const bool retryCh1Valid = isSignalFresh(retryNowUs, ch1StampUs);
    const bool retryCh2Valid = isSignalFresh(retryNowUs, ch2StampUs);
    const bool retryCh3Valid = isSignalFresh(retryNowUs, ch3StampUs);
    if (retryCh1Valid && retryCh2Valid && retryCh3Valid) {
      rcSignalLossPendingSinceMs = 0;
      rcSignalLostSinceMs = 0;
    } else {
    const uint32_t nowMs = millis();
    if (rcSignalLossPendingSinceMs == 0) {
      rcSignalLossPendingSinceMs = nowMs;
    }
    if ((nowMs - rcSignalLossPendingSinceMs) < RC_SIGNAL_LOSS_HOLD_MS) {
      return;
    }

    if (rcSignalLostSinceMs == 0) {
      rcSignalLostSinceMs = rcSignalLossPendingSinceMs;
    }
    rcNeutralReady = false;
    rcNeutralSinceMs = 0;
    rcEnableReleasedSinceReady = false;
    rcControlArmed = false;
    rcEnableActiveLatched = false;
    resetRcPivotState();
    rcThrottleIdleLatched = true;
    forceSafeStopOutputs();
    return;
    }
  }

  rcSignalLossPendingSinceMs = 0;
  rcSignalLostSinceMs = 0;
  rcLastGoodSignalMs = millis();

  if (RC_INPUT_FILTER_ENABLED) {
    rcFilteredCh1Us = filterTowardTimed(rcFilteredCh1Us, (int)ch1Us, filterDeltaUs, RC_STEERING_FILTER_US_PER_MS);
    if (rcActiveDriveChannel == DRIVE_CHANNEL_CH2) {
      rcFilteredCh2Us = filterTowardTimed(rcFilteredCh2Us, (int)ch2Us, filterDeltaUs, RC_THROTTLE_FILTER_US_PER_MS);
      rcFilteredCh3Us = filterTowardTimed(rcFilteredCh3Us, (int)ch3Us, filterDeltaUs, RC_SAFETY_FILTER_US_PER_MS);
    } else {
      rcFilteredCh2Us = filterTowardTimed(rcFilteredCh2Us, (int)ch2Us, filterDeltaUs, RC_SAFETY_FILTER_US_PER_MS);
      rcFilteredCh3Us = filterTowardTimed(rcFilteredCh3Us, (int)ch3Us, filterDeltaUs, RC_THROTTLE_FILTER_US_PER_MS);
    }
  } else {
    rcFilteredCh1Us = (int)ch1Us;
    rcFilteredCh2Us = (int)ch2Us;
    rcFilteredCh3Us = (int)ch3Us;
  }

  const bool steeringNeutral = isCenteredUs(rcFilteredCh1Us, rcSteeringCenterUs, rcSteeringReadyDeadbandUs);
  const int filteredThrottleUs = getFilteredDrivePulseUs();
  const int throttleCenterUs = getDriveChannelCenterUs();
  const bool throttleNeutral = isCenteredUs(filteredThrottleUs, throttleCenterUs, rcThrottleNeutralDeadbandUs);

  if (!rcNeutralReady) {
    if (steeringNeutral && throttleNeutral) {
      if (rcNeutralSinceMs == 0) {
        rcNeutralSinceMs = millis();
      } else if ((millis() - rcNeutralSinceMs) >= RC_NEUTRAL_CONFIRM_MS) {
        rcNeutralReady = true;
        rcEnableReleasedSinceReady = false;
        rcControlArmed = false;
        rcEnableActiveSinceReadyMs = 0;
        Serial.println("RC_SAFE_READY");
      }
    } else {
      rcNeutralSinceMs = 0;
    }

    resetRcPivotState();
    rcThrottleIdleLatched = true;
    forceSafeStopOutputs();
    return;
  }

  const uint16_t enableUs = (uint16_t)getFilteredEnablePulseUs();
  const bool safetyEnabled =
      isRcControlRequested(rcFilteredCh1Us,
                           filteredThrottleUs,
                           throttleCenterUs,
                           enableUs);
  if (!safetyEnabled) {
    rcEnableReleasedSinceReady = true;
    rcControlArmed = false;
    rcEnableActiveSinceReadyMs = 0;
    resetRcPivotState();
    rcThrottleIdleLatched = true;
    forceSafeStopOutputs();
    return;
  }

  if (!rcEnableReleasedSinceReady) {
    if (steeringNeutral && throttleNeutral) {
      if (rcEnableActiveSinceReadyMs == 0) {
        rcEnableActiveSinceReadyMs = millis();
      } else if ((millis() - rcEnableActiveSinceReadyMs) >= RC_ENABLE_AUTO_ARM_MS) {
        rcEnableReleasedSinceReady = true;
        Serial.println("RC_ENABLE_HELD_AUTO_ARM");
      }
    } else {
      rcEnableActiveSinceReadyMs = 0;
    }

    if (rcEnableReleasedSinceReady) {
      // devam edip asagida nötr arm kontrolune girelim
    } else {
    resetRcPivotState();
    rcThrottleIdleLatched = true;
    forceSafeStopOutputs();
    return;
    }
  }

  if (!rcControlArmed) {
    if (!(steeringNeutral && throttleNeutral)) {
      rcEnableActiveSinceReadyMs = 0;
      resetRcPivotState();
      rcThrottleIdleLatched = true;
      forceSafeStopOutputs();
      return;
    }
    rcControlArmed = true;
    rcEnableActiveSinceReadyMs = 0;
    Serial.println("RC_ARMED");
  }

  if (!safetyEnabled) {
    return;
  }

  int throttlePwm =
      pulseToSignedPwm((uint16_t)filteredThrottleUs,
                       throttleCenterUs,
                       rcThrottleNeutralDeadbandUs,
                       RC_THROTTLE_REVERSE);
  throttlePwm = applyGainPct(throttlePwm, RC_THROTTLE_GAIN_PCT);
  throttlePwm = clampThrottleToIdle(throttlePwm, filteredThrottleUs, throttleCenterUs);
  throttlePwm = clampSignedMagnitude(zeroSmallPwm(throttlePwm, RC_MIN_EFFECTIVE_PWM), DRIVE_MAX_PWM);
  const bool pivotTurn = (throttlePwm == 0);
  const int steeringDeadbandUs = pivotTurn ? RC_STEERING_PIVOT_DEADBAND_US : rcSteeringDriveDeadbandUs;
  const int steeringGainPct = pivotTurn ? RC_STEERING_PIVOT_GAIN_PCT : RC_STEERING_DRIVE_GAIN_PCT;
  int steeringPwm = pulseToSignedPwm((uint16_t)rcFilteredCh1Us, rcSteeringCenterUs, steeringDeadbandUs, RC_STEERING_REVERSE);

  steeringPwm = applyGainPct(steeringPwm, steeringGainPct);
  steeringPwm = zeroSmallPwm(steeringPwm, RC_MIN_EFFECTIVE_PWM);

  if (pivotTurn) {
    steeringPwm = applyPivotSteeringLatch(steeringPwm, rcFilteredCh1Us, rcSteeringCenterUs, millis());
    steeringPwm = clampi(steeringPwm, -RC_PIVOT_MAX_PWM, RC_PIVOT_MAX_PWM);
    rcPivotActive = (steeringPwm != 0);
  } else {
    resetRcPivotState();
  }

  mixRcToDrive(throttlePwm, steeringPwm, pwmL_cmd, pwmR_cmd);
  pwmL_cmd = zeroSmallPwm(pwmL_cmd, RC_MIN_EFFECTIVE_PWM);
  pwmR_cmd = zeroSmallPwm(pwmR_cmd, RC_MIN_EFFECTIVE_PWM);
  rcDebugThrottlePwm = throttlePwm;
  rcDebugSteeringPwm = steeringPwm;
  rcDebugPwmLCmd = pwmL_cmd;
  rcDebugPwmRCmd = pwmR_cmd;
  if (pwmL_cmd != 0 || pwmR_cmd != 0) {
    outputActiveStopPwmStepIntervalMs =
        (pivotTurn && (steeringPwm != 0))
            ? OUTPUT_PIVOT_STOP_PWM_STEP_INTERVAL_MS
            : OUTPUT_DRIVE_STOP_PWM_STEP_INTERVAL_MS;
  }
  lastCmdMs = millis();
}

void setup() {
  Serial.begin(115200);

  motorFR.begin();
  motorRR.begin();
  motorFL.begin();
  motorRL.begin();
  stopMotors();

  if (ENABLE_ENCODER_MONITORING) {
    encFR.begin();
    encFL.begin();
    encRL.begin();
    if (RR_ENCODER_ENABLED) {
      encRR.begin();
    }

    encFR.reset();
    encFL.reset();
    encRL.reset();
    if (RR_ENCODER_ENABLED) {
      encRR.reset();
    }
  }
  beginRawEncoderMonitor();

  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);
  pinMode(CH3_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(CH1_PIN), isrCh1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH2_PIN), isrCh2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH3_PIN), isrCh3, CHANGE);

  calibrateRcCenters();

  forceSafeStopOutputs();
  serialPwmL_cmd = 0;
  serialPwmR_cmd = 0;
  serialOverrideActive = false;
  serialLastCmdMs = 0;
  rcSignalLostSinceMs = 0;
  rcSignalLossPendingSinceMs = 0;
  rcLastGoodSignalMs = 0;
  rcLastDirectSampleMs = 0;
  rcEnableActiveLatched = false;
  rcThrottleIdleLatched = true;
  rcEnableReleasedSinceReady = false;
  rcControlArmed = false;
  rcEnableActiveSinceReadyMs = 0;
  resetRcPivotState();

  lastJsTeleMs = millis();
  lastRcTeleMs = millis();
  lastRawTeleMs = millis();
  lastModeTeleMs = millis();
  lastCmdMs  = millis();
  lastOutputUpdateMs = millis();
}

void loop() {
  pollSerialCommands();

  const uint32_t controlNowMs = millis();
  bool serialActive = isSerialOverrideActive(controlNowMs);
  bool rcModeEnabled = isRcModeEnabledNow();

  if (rcModeEnabled || !serialActive) {
    updateRcCommand();
    rcModeEnabled = isRcModeEnabledNow();
  }

  // ---- Kumanda oku (surekli dinle) ----
  if (rcModeEnabled) {
    serialOverrideActive = false;
    serialActive = false;
    resetSerialWheelControl();
  } else if (serialActive) {
    rcSignalLostSinceMs = 0;
    resetRcPivotState();
    rcThrottleIdleLatched = true;
    if (serialWheelModeActive) {
      stopDriveCommand();
      outputActiveStopPwmStepIntervalMs = OUTPUT_DRIVE_STOP_PWM_STEP_INTERVAL_MS;
    } else {
      pwmL_cmd = serialPwmL_cmd;
      pwmR_cmd = serialPwmR_cmd;
      if (pwmL_cmd != 0 || pwmR_cmd != 0) {
        const bool serialPivotCommand =
            (pwmL_cmd > 0 && pwmR_cmd < 0) || (pwmL_cmd < 0 && pwmR_cmd > 0);
        outputActiveStopPwmStepIntervalMs =
            serialPivotCommand
                ? OUTPUT_PIVOT_STOP_PWM_STEP_INTERVAL_MS
                : OUTPUT_DRIVE_STOP_PWM_STEP_INTERVAL_MS;
      }
    }
    lastCmdMs = controlNowMs;
  } else {
    rcSignalLostSinceMs = 0;
    resetRcPivotState();
    rcThrottleIdleLatched = true;
    stopDriveCommand();
  }

  clampForwardReversePwm(pwmL_cmd, pwmR_cmd);

  // ---- Timeout (opsiyonel guvenlik) ----
  if (rcModeEnabled &&
      CMD_TIMEOUT_MS > 0 &&
      (millis() - lastCmdMs) > CMD_TIMEOUT_MS) {
    stopDriveCommand();
  }

  // ---- Encoder update ----
  if (ENABLE_ENCODER_MONITORING) {
    encFR.update();
    encFL.update();
    encRL.update();
    if (RR_ENCODER_ENABLED) {
      encRR.update();
    }
  }
  if (ENABLE_RAW_ENCODER_TELEMETRY) {
    updateRawEncoderMonitor();
  }

  // ---- Motor cikislarini korumali uygula ----
  if (serialActive && serialWheelModeActive) {
    applySerialWheelTestOutputs();
  } else {
    applyDriveOutputs();
  }

  // ---- Telemetri ----
  uint32_t nowMs = millis();
  uint32_t t_us = micros();
  uint16_t ch1Us = 0;
  uint16_t ch2Us = 0;
  uint16_t ch3Us = 0;
  uint32_t ch1StampUs = 0;
  uint32_t ch2StampUs = 0;
  uint32_t ch3StampUs = 0;

  readRcSnapshot(ch1Us, ch2Us, ch3Us, ch1StampUs, ch2StampUs, ch3StampUs);

  const bool ch1Valid = isSignalFresh(t_us, ch1StampUs);
  const bool ch2Valid = isSignalFresh(t_us, ch2StampUs);
  const bool ch3Valid = isSignalFresh(t_us, ch3StampUs);

  if (nowMs - lastJsTeleMs >= JS_TELEMETRY_MS) {
    lastJsTeleMs = nowMs;

    // JS:
    // compact:
    // seq t_us FLrpm10 FRrpm10 RLrpm10 RRrpm10 pwmL pwmR FLcmps10 FRcmps10 RLcmps10 RRcmps10 RobotCmps10
    const float flRpm = ENABLE_ENCODER_MONITORING ? encFL.getRPM() : 0.0f;
    const float frRpm = ENABLE_ENCODER_MONITORING ? encFR.getRPM() : 0.0f;
    const float rlRpm = ENABLE_ENCODER_MONITORING ? encRL.getRPM() : 0.0f;
    const float rrRpm = (ENABLE_ENCODER_MONITORING && RR_ENCODER_ENABLED) ? encRR.getRPM() : 0.0f;

    const long flRpm10 = floatToSigned10(flRpm);
    const long frRpm10 = floatToSigned10(frRpm);
    const long rlRpm10 = floatToSigned10(rlRpm);
    const long rrRpm10 = floatToSigned10(rrRpm);

    const float flSpeedCmS = rpmToWheelSpeedCmS(flRpm);
    const float frSpeedCmS = rpmToWheelSpeedCmS(frRpm);
    const float rlSpeedCmS = rpmToWheelSpeedCmS(rlRpm);
    const float rrSpeedCmS = rpmToWheelSpeedCmS(rrRpm);

    const long flSpeedCmS10 = floatToSigned10(flSpeedCmS);
    const long frSpeedCmS10 = floatToSigned10(frSpeedCmS);
    const long rlSpeedCmS10 = floatToSigned10(rlSpeedCmS);
    const long rrSpeedCmS10 = floatToSigned10(rrSpeedCmS);

    float robotSpeedCmS = 0.0f;
    if (ENABLE_ENCODER_MONITORING) {
      if (RR_ENCODER_ENABLED) {
        robotSpeedCmS = 0.25f * (flSpeedCmS + frSpeedCmS + rlSpeedCmS + rrSpeedCmS);
      } else {
        robotSpeedCmS = (flSpeedCmS + frSpeedCmS + rlSpeedCmS) / 3.0f;
      }
    }
    const long robotSpeedCmS10 = floatToSigned10(robotSpeedCmS);

    if (JS_COMPACT_TELEMETRY) {
      char jsLine[SERIAL_JS_BUFFER_BYTES];
      const int jsLen = snprintf(jsLine,
                                 sizeof(jsLine),
                                 "JS %lu %lu %ld %ld %ld %ld %d %d %ld %ld %ld %ld %ld\n",
                                 (unsigned long)seq,
                                 (unsigned long)t_us,
                                 flRpm10,
                                 frRpm10,
                                 rlRpm10,
                                 rrRpm10,
                                 pwmL_out,
                                 pwmR_out,
                                 flSpeedCmS10,
                                 frSpeedCmS10,
                                 rlSpeedCmS10,
                                 rrSpeedCmS10,
                                 robotSpeedCmS10);
      if (jsLen > 0 && jsLen < (int)sizeof(jsLine)) {
        serialWriteLineIfFits(jsLine, (size_t)jsLen);
      }
    } else {
      // verbose:
      // seq t_us FLpos FRpos RLpos RRpos FLvel FRvel RLvel RRvel FLcmps FRcmps RLcmps RRcmps RobotCmps FLticks FRticks RLticks RRticks pwmL pwmR
      Serial.print("JS ");
      Serial.print(seq); Serial.print(' ');
      Serial.print(t_us); Serial.print(' ');

      Serial.print(encFL.getPositionRad(), 4); Serial.print(' ');
      Serial.print(encFR.getPositionRad(), 4); Serial.print(' ');
      Serial.print(encRL.getPositionRad(), 4); Serial.print(' ');
      Serial.print(RR_ENCODER_ENABLED ? encRR.getPositionRad() : 0.0, 4); Serial.print(' ');

      Serial.print(encFL.getVelocityRadS(), 3); Serial.print(' ');
      Serial.print(encFR.getVelocityRadS(), 3); Serial.print(' ');
      Serial.print(encRL.getVelocityRadS(), 3); Serial.print(' ');
      Serial.print(RR_ENCODER_ENABLED ? encRR.getVelocityRadS() : 0.0, 3); Serial.print(' ');

      Serial.print(flSpeedCmS, 2); Serial.print(' ');
      Serial.print(frSpeedCmS, 2); Serial.print(' ');
      Serial.print(rlSpeedCmS, 2); Serial.print(' ');
      Serial.print(RR_ENCODER_ENABLED ? rrSpeedCmS : 0.0, 2); Serial.print(' ');
      Serial.print(robotSpeedCmS, 2); Serial.print(' ');

      Serial.print(encFL.getTicks()); Serial.print(' ');
      Serial.print(encFR.getTicks()); Serial.print(' ');
      Serial.print(encRL.getTicks()); Serial.print(' ');
      Serial.print(RR_ENCODER_ENABLED ? encRR.getTicks() : 0); Serial.print(' ');

      Serial.print(pwmL_out); Serial.print(' ');
      Serial.print(pwmR_out);
      Serial.println();
    }

    seq++;
  }

  if (nowMs - lastRcTeleMs >= RC_TELEMETRY_MS) {
    lastRcTeleMs = nowMs;

    // RC:
    // ch1_us ch2_us ch3_us ch1_ok ch2_ok ch3_ok
    char rcLine[SERIAL_RC_BUFFER_BYTES];
    const int rcLen = snprintf(rcLine,
                               sizeof(rcLine),
                               "RC %u %u %u %d %d %d\n",
                               (unsigned int)ch1Us,
                               (unsigned int)ch2Us,
                               (unsigned int)ch3Us,
                               ch1Valid ? 1 : 0,
                               ch2Valid ? 1 : 0,
                               ch3Valid ? 1 : 0);
    if (rcLen > 0 && rcLen < (int)sizeof(rcLine)) {
      serialWriteLineIfFits(rcLine, (size_t)rcLen);
    }

    char rcDbgLine[120];
    const int rcDbgLen = snprintf(rcDbgLine,
                                  sizeof(rcDbgLine),
                                  "RCDBG %d %d %d %d %d %d %d %d %d %d\n",
                                  rcDebugThrottlePwm,
                                  rcDebugSteeringPwm,
                                  rcDebugPwmLCmd,
                                  rcDebugPwmRCmd,
                                  pwmL_out,
                                  pwmR_out,
                                  rcNeutralReady ? 1 : 0,
                                  rcControlArmed ? 1 : 0,
                                  rcEnableReleasedSinceReady ? 1 : 0,
                                  rcEnableActiveLatched ? 1 : 0);
    if (rcDbgLen > 0 && rcDbgLen < (int)sizeof(rcDbgLine)) {
      serialWriteLineIfFits(rcDbgLine, (size_t)rcDbgLen);
    }

    if (serialActive && serialWheelModeActive) {
      const float teleFlRpm = ENABLE_ENCODER_MONITORING ? encFL.getRPM() : 0.0f;
      const float teleFrRpm = ENABLE_ENCODER_MONITORING ? encFR.getRPM() : 0.0f;
      const float teleRlRpm = ENABLE_ENCODER_MONITORING ? encRL.getRPM() : 0.0f;
      const float teleRrRpm =
          (ENABLE_ENCODER_MONITORING && RR_ENCODER_ENABLED) ? encRR.getRPM() : 0.0f;
      char wtestLine[SERIAL_WTEST_BUFFER_BYTES];
      const int wtestLen = snprintf(wtestLine,
                                    sizeof(wtestLine),
                                    "WTEST %s %d %ld %d %ld %ld %ld %ld\n",
                                    getSerialWheelName(serialWheelId),
                                    serialWheelTargetRpmCmd,
                                    floatToSigned10(serialWheelMeasuredRpm),
                                    serialWheelPwmOut,
                                    floatToSigned10(teleFlRpm),
                                    floatToSigned10(teleFrRpm),
                                    floatToSigned10(teleRlRpm),
                                    floatToSigned10(teleRrRpm));
      if (wtestLen > 0 && wtestLen < (int)sizeof(wtestLine)) {
        serialWriteLineIfFits(wtestLine, (size_t)wtestLen);
      }
    }
  }

  if (ENABLE_RAW_ENCODER_TELEMETRY &&
      (nowMs - lastRawTeleMs >= RAW_TELEMETRY_MS)) {
    lastRawTeleMs = nowMs;

    // ENC_RAW:
    // FL_A FL_B FL_edges FR_A FR_B FR_edges RL_A RL_B RL_edges RR_A RR_B RR_edges
    char rawLine[SERIAL_RAW_BUFFER_BYTES];
    const int rawLen = snprintf(rawLine,
                                sizeof(rawLine),
                                "ENC_RAW %u %u %lu %u %u %lu %u %u %lu %u %u %lu\n",
                                (unsigned int)rawFlA,
                                (unsigned int)rawFlB,
                                (unsigned long)rawFlEdges,
                                (unsigned int)rawFrA,
                                (unsigned int)rawFrB,
                                (unsigned long)rawFrEdges,
                                (unsigned int)rawRlA,
                                (unsigned int)rawRlB,
                                (unsigned long)rawRlEdges,
                                (unsigned int)rawRrA,
                                (unsigned int)rawRrB,
                                (unsigned long)rawRrEdges);
    if (rawLen > 0 && rawLen < (int)sizeof(rawLine)) {
      serialWriteLineIfFits(rawLine, (size_t)rawLen);
    }
  }

  if (nowMs - lastModeTeleMs >= MODE_TELEMETRY_MS) {
    lastModeTeleMs = nowMs;

    const char* modeText = "BEKLEME";
    if (rcModeEnabled) {
      modeText = "RC_KUMANDA_AKTIF";
    } else if (serialActive && serialWheelModeActive) {
      modeText = "SERI_WHEEL_AKTIF";
    } else if (serialActive) {
      modeText = "SERI_PWM_AKTIF";
    }

    char modeLine[32];
    const int modeLen = snprintf(modeLine,
                                 sizeof(modeLine),
                                 "MODE %s\n",
                                 modeText);
    if (modeLen > 0 && modeLen < (int)sizeof(modeLine)) {
      serialWriteLineIfFits(modeLine, (size_t)modeLen);
    }

    const int serialUserL = serialPwmL_cmd * SERIAL_PWM_SIGN;
    const int serialUserR = serialPwmR_cmd * SERIAL_PWM_SIGN;
    const char* cmdDirText = "IDLE";
    if (rcModeEnabled) {
      cmdDirText = "RC_MODE";
    } else if (serialActive && serialWheelModeActive) {
      cmdDirText = getSerialWheelName(serialWheelId);
    } else if (serialActive) {
      cmdDirText = classifyUserSerialDirection(serialUserL, serialUserR);
    }

    char cmdLine[32];
    const int cmdLen = snprintf(cmdLine,
                                sizeof(cmdLine),
                                "CMD_DIR %s\n",
                                cmdDirText);
    if (cmdLen > 0 && cmdLen < (int)sizeof(cmdLine)) {
      serialWriteLineIfFits(cmdLine, (size_t)cmdLen);
    }
  }

}
