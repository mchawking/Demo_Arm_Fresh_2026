#include "demo_arm.h"

#include <Arduino.h>
#include <Wire.h>
#include <driver/twai.h>
#include <string.h>
#include <stdlib.h>

/*
CALIBRATION QUICK PROCEDURE (Receiver Demo Arm)

Goal:
- Teach each axis MIN, MAX, and HOME using live encoder feedback.
- Save calibration to FRAM so it is restored on next boot.

Setup:
- Power receiver and open serial monitor at 115200.
- Confirm startup prints FRAM detect/self-test status.

Enter calibration mode (choose one):
1) Hold CAL button for 5 seconds.
2) Or type: CAL START

Per-axis capture sequence (repeat for axis 0, 1, 2):
1) Select axis: CAL NODE 0   (or 1 / 2)
2) Capture MIN:
  - Move joint so turns decrease (more negative), then run CAL CAP
3) Capture MAX:
  - Move opposite direction so turns increase (more positive), then CAL CAP
4) Capture HOME:
  - Move to neutral between MIN and MAX, then CAL CAP

Verify while moving (optional):
- CAL STATUS     (shows turns, velocity, direction)
- CAL FEED ON    (live motion feed)
- CAL FEED OFF

Save and exit:
1) Serial path: CAL SAVE
2) Button path: hold CAL button 5 seconds (saves only after complete capture)

Exit without saving:
- CAL EXIT

Notes:
- Valid order must be: MIN < HOME < MAX.
- During calibration, position commands are not sent (manual hand positioning).
- On exit, axes ramp back toward HOME in normal control mode.
*/

// CRSF receiver wiring
#define CRSF_SERIAL       Serial1
#define CRSF_RX_PIN       16
#define CRSF_TX_PIN       17
#define CRSF_BAUD         420000

// CAN transceiver wiring / bus config
#define CAN_RX_PIN       GPIO_NUM_10  // previously 21
#define CAN_TX_PIN       GPIO_NUM_11  // previously 18
#define ODRIVE_CAN_BAUD  250000

// ODrive CANSimple command IDs
#define ODRIVE_CMD_GET_ENCODER_ESTIMATES 0x09
#define ODRIVE_CMD_SET_AXIS_STATE       0x07
#define ODRIVE_CMD_SET_CONTROLLER_MODES 0x0B
#define ODRIVE_CMD_SET_INPUT_POS        0x0C

#define ODRIVE_AXIS_STATE_IDLE          1u
#define ODRIVE_AXIS_STATE_CLOSED_LOOP   8u
#define ODRIVE_CONTROL_MODE_POSITION    3u
#define ODRIVE_INPUT_MODE_PASSTHROUGH   1u

// CRSF protocol constants
#define CRSF_SYNC               0xC8
#define CRSF_TYPE_RC_CHANNELS   0x16
#define CRSF_RC_PAYLOAD_LEN     22
#define CRSF_MAX_FRAME          64
#define SIGNAL_TIMEOUT_MS       1000

// Runtime timing
#define ODRIVE_SEND_PERIOD_MS   20

// Safety / controls
#define ARM_SWITCH_CHANNEL      5
#define ARM_THRESHOLD           1200
#define SAFE_RETURN_MS          5000

// Button: hold for 5s toggles calibration mode, short press captures point
#define CAL_BUTTON_PIN          4
#define CAL_BUTTON_HOLD_MS      5000
#define CAL_DIR_VEL_DEADBAND    0.01f
#define CAL_FEED_PERIOD_MS      250
#define CAL_FEED_TURN_DELTA     0.001f

// FRAM (MB85RC-style over I2C)
#define FRAM_I2C_ADDRESS        0x50
#define FRAM_BASE_ADDRESS       0x0000
#define FRAM_I2C_SDA_PIN        8
#define FRAM_I2C_SCL_PIN        9
#define FRAM_SIZE_BYTES         32768u
#define FRAM_SELFTEST_ADDR      (FRAM_SIZE_BYTES - 2u)

// Transmitter calibrated domain. Adjust to your TX calibration if needed.
#define TX_CAL_MIN              172
#define TX_CAL_CENTER           992
#define TX_CAL_MAX              1811

struct AxisBinding {
  uint8_t nodeId;
  uint8_t channel;      // CRSF channel 1..16
};

struct AxisCalibration {
  float minTurns;
  float maxTurns;
  float homeTurns;
};

struct AxisRuntime {
  float commandedTurns;
  float measuredTurns;
  float measuredVel;
  bool measuredValid;
  float rampStartTurns;
  unsigned long rampStartMs;
  bool rampingToHome;
};

struct CalibrationBlob {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  AxisCalibration axes[3];
  uint32_t crc;
};

static const uint32_t CALIB_MAGIC = 0x43414C42; // 'CALB'
static const uint16_t CALIB_VERSION = 1;

static const AxisBinding gAxes[3] = {
  {0, 1}, // Node 0 shoulder <- CH1
  {1, 2}, // Node 1 upper arm <- CH2
  {2, 3}, // Node 2 lower arm <- CH3
};

// Safe defaults used before calibration exists or if FRAM data is invalid.
// Shoulder allows multi-turn due to gearbox.
// Upper/lower are limited to +/-45 deg from home (+/-0.125 turns).
static const AxisCalibration gDefaultCal[3] = {
  {-2.0f, 2.0f, 0.0f},
  {-0.125f, 0.125f, 0.0f},
  {-0.125f, 0.125f, 0.0f},
};

// Wider temporary ranges while capturing calibration points.
static const AxisCalibration gTeachRange[3] = {
  {-3.0f, 3.0f, 0.0f},
  {-0.5f, 0.5f, 0.0f},
  {-0.5f, 0.5f, 0.0f},
};

static AxisCalibration gCal[3];
static AxisRuntime gRt[3];

static uint16_t      _ch[16];
static bool          _hasSignal = false;
static unsigned long _lastFrameMs = 0;
static uint8_t       _rxBuf[CRSF_MAX_FRAME];
static uint8_t       _rxLen = 0;

static bool          _canReady = false;
static bool          _prevArmed = false;
static unsigned long _lastOdriveSendMs = 0;

static bool          gFramDetected = false;
static bool          gFramSelfTestOk = false;

static bool          gCalibrationMode = false;
static uint8_t       gCalAxis = 0;     // 0..2
static uint8_t       gCalPoint = 0;    // 0=min, 1=max, 2=home
static bool          gCalComplete = false;
static bool          gCalAxisSelected = false;
static bool          gCalLiveFeed = true;
static unsigned long gCalLiveLastMs = 0;
static bool          gCalFeedHasLast = false;
static float         gCalFeedLastTurns = 0.0f;
static int8_t        gCalFeedLastDir = 0;

static bool          gBtnPrevPressed = false;
static bool          gBtnLongHandled = false;
static unsigned long gBtnPressStartMs = 0;

static char          gSerialCmdBuf[48];
static uint8_t       gSerialCmdLen = 0;

static float clampf(float value, float minV, float maxV) {
  if (value < minV) return minV;
  if (value > maxV) return maxV;
  return value;
}

static bool isValidAxisCal(const AxisCalibration& c) {
  return (c.minTurns < c.homeTurns) && (c.homeTurns < c.maxTurns);
}

static bool isValidAllCal(const AxisCalibration* cals) {
  for (int i = 0; i < 3; i++) {
    if (!isValidAxisCal(cals[i])) {
      return false;
    }
  }
  return true;
}

static uint32_t fnv1a32(const uint8_t* data, size_t len) {
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < len; i++) {
    hash ^= data[i];
    hash *= 16777619u;
  }
  return hash;
}

static bool framWriteByte(uint16_t addr, uint8_t value) {
  Wire.beginTransmission(FRAM_I2C_ADDRESS);
  Wire.write((uint8_t)(addr >> 8));
  Wire.write((uint8_t)(addr & 0xFF));
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool framReadByte(uint16_t addr, uint8_t* out) {
  Wire.beginTransmission(FRAM_I2C_ADDRESS);
  Wire.write((uint8_t)(addr >> 8));
  Wire.write((uint8_t)(addr & 0xFF));
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom((int)FRAM_I2C_ADDRESS, 1) != 1) {
    return false;
  }

  *out = Wire.read();
  return true;
}

static bool framWriteBytes(uint16_t addr, const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (!framWriteByte((uint16_t)(addr + i), data[i])) {
      return false;
    }
  }
  return true;
}

static bool framReadBytes(uint16_t addr, uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (!framReadByte((uint16_t)(addr + i), &data[i])) {
      return false;
    }
  }
  return true;
}

static bool framProbe() {
  Wire.beginTransmission(FRAM_I2C_ADDRESS);
  return Wire.endTransmission() == 0;
}

// Non-destructive FRAM test: backup -> write pattern -> verify -> restore.
static bool framSelfTest() {
  uint8_t original[2] = {0, 0};
  uint8_t pattern[2] = {0xA5, 0x5A};
  uint8_t verify[2] = {0, 0};

  if (!framReadBytes(FRAM_SELFTEST_ADDR, original, sizeof(original))) {
    return false;
  }
  if (!framWriteBytes(FRAM_SELFTEST_ADDR, pattern, sizeof(pattern))) {
    return false;
  }
  if (!framReadBytes(FRAM_SELFTEST_ADDR, verify, sizeof(verify))) {
    return false;
  }

  bool ok = (verify[0] == pattern[0] && verify[1] == pattern[1]);

  // Always attempt restore so test does not alter persistent state.
  framWriteBytes(FRAM_SELFTEST_ADDR, original, sizeof(original));
  return ok;
}

static bool loadCalibrationFromFram() {
  CalibrationBlob blob = {};
  if (!framReadBytes(FRAM_BASE_ADDRESS, (uint8_t*)&blob, sizeof(blob))) {
    return false;
  }

  if (blob.magic != CALIB_MAGIC || blob.version != CALIB_VERSION || blob.size != sizeof(CalibrationBlob)) {
    return false;
  }

  uint32_t expected = fnv1a32((const uint8_t*)&blob, sizeof(blob) - sizeof(blob.crc));
  if (expected != blob.crc) {
    return false;
  }

  if (!isValidAllCal(blob.axes)) {
    return false;
  }

  for (int i = 0; i < 3; i++) {
    gCal[i] = blob.axes[i];
  }
  return true;
}

static bool saveCalibrationToFram() {
  if (!isValidAllCal(gCal)) {
    return false;
  }

  CalibrationBlob blob = {};
  blob.magic = CALIB_MAGIC;
  blob.version = CALIB_VERSION;
  blob.size = sizeof(CalibrationBlob);
  for (int i = 0; i < 3; i++) {
    blob.axes[i] = gCal[i];
  }
  blob.crc = fnv1a32((const uint8_t*)&blob, sizeof(blob) - sizeof(blob.crc));

  return framWriteBytes(FRAM_BASE_ADDRESS, (const uint8_t*)&blob, sizeof(blob));
}

static uint8_t crc8(uint8_t crc, uint8_t data) {
  crc ^= data;
  for (int i = 0; i < 8; i++) {
    crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  }
  return crc;
}

static void unpackChannels(const uint8_t *p) {
  _ch[0]  = (p[0]        | p[1]  << 8)  & 0x7FF;
  _ch[1]  = (p[1]  >> 3  | p[2]  << 5)  & 0x7FF;
  _ch[2]  = (p[2]  >> 6  | p[3]  << 2   | p[4]  << 10) & 0x7FF;
  _ch[3]  = (p[4]  >> 1  | p[5]  << 7)  & 0x7FF;
  _ch[4]  = (p[5]  >> 4  | p[6]  << 4)  & 0x7FF;
  _ch[5]  = (p[6]  >> 7  | p[7]  << 1   | p[8]  << 9)  & 0x7FF;
  _ch[6]  = (p[8]  >> 2  | p[9]  << 6)  & 0x7FF;
  _ch[7]  = (p[9]  >> 5  | p[10] << 3)  & 0x7FF;
  _ch[8]  = (p[11]       | p[12] << 8)  & 0x7FF;
  _ch[9]  = (p[12] >> 3  | p[13] << 5)  & 0x7FF;
  _ch[10] = (p[13] >> 6  | p[14] << 2   | p[15] << 10) & 0x7FF;
  _ch[11] = (p[15] >> 1  | p[16] << 7)  & 0x7FF;
  _ch[12] = (p[16] >> 4  | p[17] << 4)  & 0x7FF;
  _ch[13] = (p[17] >> 7  | p[18] << 1   | p[19] << 9)  & 0x7FF;
  _ch[14] = (p[19] >> 2  | p[20] << 6)  & 0x7FF;
  _ch[15] = (p[20] >> 5  | p[21] << 3)  & 0x7FF;
}

static void processCrsf() {
  while (CRSF_SERIAL.available()) {
    uint8_t b = CRSF_SERIAL.read();

    if (_rxLen == 0 && b != CRSF_SYNC) continue;

    _rxBuf[_rxLen++] = b;
    if (_rxLen < 3) continue;

    uint8_t frameLen = _rxBuf[1];
    uint8_t totalLen = (uint8_t)(2 + frameLen);

    if (totalLen > CRSF_MAX_FRAME) {
      _rxLen = 0;
      continue;
    }
    if (_rxLen < totalLen) continue;

    uint8_t crc = 0;
    for (int i = 2; i < totalLen - 1; i++) crc = crc8(crc, _rxBuf[i]);

    if (crc == _rxBuf[totalLen - 1] &&
        _rxBuf[2] == CRSF_TYPE_RC_CHANNELS &&
        (frameLen - 2) == CRSF_RC_PAYLOAD_LEN) {
      unpackChannels(&_rxBuf[3]);
      _hasSignal = true;
      _lastFrameMs = millis();
    }

    _rxLen = 0;
  }

  if (_hasSignal && (millis() - _lastFrameMs > SIGNAL_TIMEOUT_MS)) {
    _hasSignal = false;
  }
}

static uint32_t buildArbitrationId(uint8_t nodeId, uint8_t cmdId) {
  return ((uint32_t)nodeId << 5) | cmdId;
}

static bool sendCanFrame(uint8_t nodeId, uint8_t cmdId, const void* payload, uint8_t dlc) {
  twai_message_t msg = {};
  msg.identifier = buildArbitrationId(nodeId, cmdId);
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = dlc;
  if (payload != nullptr && dlc > 0) {
    memcpy(msg.data, payload, dlc);
  }
  return twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK;
}

static bool requestCanFrame(uint8_t nodeId, uint8_t cmdId) {
  twai_message_t msg = {};
  msg.identifier = buildArbitrationId(nodeId, cmdId);
  msg.extd = 0;
  msg.rtr = 1;
  msg.data_length_code = 8;
  return twai_transmit(&msg, pdMS_TO_TICKS(10)) == ESP_OK;
}

static bool sendOdriveSetState(uint8_t nodeId, uint32_t axisState) {
  return sendCanFrame(nodeId, ODRIVE_CMD_SET_AXIS_STATE, &axisState, 4);
}

static bool sendOdriveControllerModes(uint8_t nodeId, uint32_t controlMode, uint32_t inputMode) {
  uint8_t payload[8] = {};
  memcpy(&payload[0], &controlMode, 4);
  memcpy(&payload[4], &inputMode, 4);
  return sendCanFrame(nodeId, ODRIVE_CMD_SET_CONTROLLER_MODES, payload, 8);
}

static bool sendOdrivePosition(uint8_t nodeId, float positionTurns) {
  uint8_t payload[8] = {};
  int16_t velFf = 0;
  int16_t torqueFf = 0;
  memcpy(&payload[0], &positionTurns, 4);
  memcpy(&payload[4], &velFf, 2);
  memcpy(&payload[6], &torqueFf, 2);
  return sendCanFrame(nodeId, ODRIVE_CMD_SET_INPUT_POS, payload, 8);
}

static bool initCanBus() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t;
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  switch (ODRIVE_CAN_BAUD) {
    case 1000000: t = TWAI_TIMING_CONFIG_1MBITS(); break;
    case 500000:  t = TWAI_TIMING_CONFIG_500KBITS(); break;
    case 250000:  t = TWAI_TIMING_CONFIG_250KBITS(); break;
    case 125000:  t = TWAI_TIMING_CONFIG_125KBITS(); break;
    default: return false;
  }

  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  if (twai_start() != ESP_OK) {
    twai_driver_uninstall();
    return false;
  }
  return true;
}

static void processCanRx() {
  twai_message_t rx = {};
  while (twai_receive(&rx, 0) == ESP_OK) {
    uint8_t nodeId = (uint8_t)(rx.identifier >> 5);
    uint8_t cmdId = (uint8_t)(rx.identifier & 0x1F);

    if (cmdId != ODRIVE_CMD_GET_ENCODER_ESTIMATES || rx.data_length_code < 8) {
      continue;
    }

    for (int i = 0; i < 3; i++) {
      if (gAxes[i].nodeId != nodeId) {
        continue;
      }

      memcpy(&gRt[i].measuredTurns, &rx.data[0], 4);
      memcpy(&gRt[i].measuredVel, &rx.data[4], 4);
      gRt[i].measuredValid = true;
      break;
    }
  }
}

static float mapChannelToTurns(uint16_t raw, const AxisCalibration& cal) {
  if (raw <= TX_CAL_CENTER) {
    float t = (float)(raw - TX_CAL_MIN) / (float)(TX_CAL_CENTER - TX_CAL_MIN);
    t = clampf(t, 0.0f, 1.0f);
    return cal.minTurns + t * (cal.homeTurns - cal.minTurns);
  }

  float t = (float)(raw - TX_CAL_CENTER) / (float)(TX_CAL_MAX - TX_CAL_CENTER);
  t = clampf(t, 0.0f, 1.0f);
  return cal.homeTurns + t * (cal.maxTurns - cal.homeTurns);
}

static float getRampToHome(const AxisCalibration& cal, AxisRuntime& rt, unsigned long nowMs) {
  if (!rt.rampingToHome) {
    return cal.homeTurns;
  }

  unsigned long elapsed = nowMs - rt.rampStartMs;
  if (elapsed >= SAFE_RETURN_MS) {
    rt.rampingToHome = false;
    return cal.homeTurns;
  }

  float alpha = (float)elapsed / (float)SAFE_RETURN_MS;
  return rt.rampStartTurns + (cal.homeTurns - rt.rampStartTurns) * alpha;
}

static void enterCalibrationMode() {
  gCalibrationMode = true;
  gCalAxis = 0;
  gCalPoint = 0;
  gCalComplete = false;
  gCalAxisSelected = false;
  gCalLiveFeed = true;
  gCalLiveLastMs = 0;
  gCalFeedHasLast = false;
  gCalFeedLastTurns = 0.0f;
  gCalFeedLastDir = 0;
  _prevArmed = false;

  for (int i = 0; i < 3; i++) {
    sendOdriveControllerModes(gAxes[i].nodeId, ODRIVE_CONTROL_MODE_POSITION, ODRIVE_INPUT_MODE_PASSTHROUGH);
    sendOdriveSetState(gAxes[i].nodeId, ODRIVE_AXIS_STATE_CLOSED_LOOP);
    gRt[i].rampingToHome = false;
  }

  Serial.println("\n=== CALIBRATION MODE ===");
  Serial.println("Short press button to capture each point: MIN -> MAX -> HOME");
  Serial.println("Hold button 5s to save and exit calibration mode");
  Serial.println("Select axis first: CAL NODE 0, CAL NODE 1, or CAL NODE 2");
  Serial.println("Live feed is ON: shows turns/vel/dir while calibrating selected axis");
}

static void exitCalibrationMode(bool saveToFram) {
  gCalibrationMode = false;
  _prevArmed = false;

  if (saveToFram) {
    if (isValidAllCal(gCal) && saveCalibrationToFram()) {
      Serial.println("Calibration saved to FRAM");
    } else {
      Serial.println("Calibration not saved (invalid data or FRAM write failure)");
    }
  } else {
    Serial.println("Calibration exited without saving");
  }

  for (int i = 0; i < 3; i++) {
    gRt[i].rampStartTurns = gRt[i].commandedTurns;
    gRt[i].rampStartMs = millis();
    gRt[i].rampingToHome = true;
  }

  Serial.println("=== NORMAL MODE ===\n");
}

static const char* calPointName(uint8_t point) {
  if (point == 0) return "MIN";
  if (point == 1) return "MAX";
  return "HOME";
}

static const char* directionFromVelocity(float velocity) {
  if (velocity > CAL_DIR_VEL_DEADBAND) return "POSITIVE";
  if (velocity < -CAL_DIR_VEL_DEADBAND) return "NEGATIVE";
  return "STILL";
}

static int8_t directionCodeFromVelocity(float velocity) {
  if (velocity > CAL_DIR_VEL_DEADBAND) return 1;
  if (velocity < -CAL_DIR_VEL_DEADBAND) return -1;
  return 0;
}

static void printAxisSaveCheck(uint8_t axis) {
  if (axis >= 3) return;

  const AxisCalibration& c = gCal[axis];
  bool pass = isValidAxisCal(c);

  Serial.printf("SAVE CHECK Axis %u: MIN=%.4f HOME=%.4f MAX=%.4f => %s\n",
                axis,
                c.minTurns,
                c.homeTurns,
                c.maxTurns,
                pass ? "PASS" : "FAIL");

  if (!pass) {
    if (!(c.minTurns < c.homeTurns)) {
      Serial.println("  FAIL reason: MIN must be less than HOME");
    }
    if (!(c.homeTurns < c.maxTurns)) {
      Serial.println("  FAIL reason: HOME must be less than MAX");
    }
  }
}

static void printSaveReadinessSummary() {
  Serial.println("=== PRE-SAVE SUMMARY ===");

  if (gCalAxisSelected && gCalAxis < 3) {
    printAxisSaveCheck(gCalAxis);
  } else {
    Serial.println("Selected axis: none");
  }

  bool allValid = isValidAllCal(gCal);
  Serial.printf("ALL AXES SAVE READINESS: %s\n", allValid ? "PASS" : "FAIL");
  if (!allValid) {
    for (uint8_t i = 0; i < 3; i++) {
      if (!isValidAxisCal(gCal[i])) {
        printAxisSaveCheck(i);
      }
    }
  }
  Serial.println("========================");
}

static void printCalibrationStepPrompt() {
  if (!gCalibrationMode) return;

  if (!gCalAxisSelected) {
    Serial.println("Select axis first: CAL NODE 0, CAL NODE 1, or CAL NODE 2");
    return;
  }

  if (gCalAxis >= 3) return;

  if (gCalPoint == 0) {
    Serial.printf("STEP 1/3 Axis %u MIN: turn joint so measured turns DECREASE (more negative), then CAL CAP\n", gCalAxis);
  } else if (gCalPoint == 1) {
    Serial.printf("STEP 2/3 Axis %u MAX: turn joint opposite direction so measured turns INCREASE (more positive), then CAL CAP\n", gCalAxis);
    Serial.printf("  Keep moving until value is greater than MIN %.4f\n", gCal[gCalAxis].minTurns);
  } else {
    Serial.printf("STEP 3/3 Axis %u HOME: move to neutral between MIN and MAX, then CAL CAP\n", gCalAxis);
    Serial.printf("  Target between %.4f and %.4f\n", gCal[gCalAxis].minTurns, gCal[gCalAxis].maxTurns);
  }

  Serial.println("  Direction check: POSITIVE means turns increasing; NEGATIVE means turns decreasing.");

  if (gRt[gCalAxis].measuredValid) {
    Serial.printf("  Current measured turns: %.4f | vel: %.4f | dir: %s\n",
                  gRt[gCalAxis].measuredTurns,
                  gRt[gCalAxis].measuredVel,
                  directionFromVelocity(gRt[gCalAxis].measuredVel));
    Serial.println("  Use CAL STATUS while moving to confirm direction sign.");
  } else {
    Serial.println("  Waiting for encoder feedback... (use CAL STATUS)");
  }
}

static void printCalibrationLiveFeed(unsigned long nowMs, bool force) {
  if (!gCalibrationMode || !gCalAxisSelected || gCalAxis >= 3) {
    return;
  }

  if (!gCalLiveFeed && !force) {
    return;
  }

  if (!force && (nowMs - gCalLiveLastMs < CAL_FEED_PERIOD_MS)) {
    return;
  }

  gCalLiveLastMs = nowMs;

  if (!gRt[gCalAxis].measuredValid) {
    return;
  }

  int8_t dirCode = directionCodeFromVelocity(gRt[gCalAxis].measuredVel);
  if (dirCode == 0) {
    return;
  }

  bool shouldPrint = force || !gCalFeedHasLast;
  if (!shouldPrint) {
    float delta = gRt[gCalAxis].measuredTurns - gCalFeedLastTurns;
    if (delta < 0.0f) {
      delta = -delta;
    }

    if (delta >= CAL_FEED_TURN_DELTA) {
      shouldPrint = true;
    }
  }

  if (!shouldPrint) {
    return;
  }

  gCalFeedHasLast = true;
  gCalFeedLastTurns = gRt[gCalAxis].measuredTurns;
  gCalFeedLastDir = dirCode;

  Serial.printf("LIVE Axis %u point=%s turns=%.4f vel=%.4f dir=%s\n",
                gCalAxis,
                calPointName(gCalPoint),
                gRt[gCalAxis].measuredTurns,
                gRt[gCalAxis].measuredVel,
                directionFromVelocity(gRt[gCalAxis].measuredVel));
}

static bool selectCalibrationAxis(int axis) {
  if (axis < 0 || axis > 2) {
    Serial.println("Invalid axis. Use CAL NODE 0, CAL NODE 1, or CAL NODE 2.");
    return false;
  }

  gCalAxis = (uint8_t)axis;
  gCalPoint = 0;
  gCalComplete = false;
  gCalAxisSelected = true;
  gCalFeedHasLast = false;
  gCalFeedLastTurns = 0.0f;
  gCalFeedLastDir = 0;

  // In calibration mode, release selected axis torque so it can be moved by hand.
  sendOdriveSetState(gAxes[gCalAxis].nodeId, ODRIVE_AXIS_STATE_IDLE);

  Serial.printf("Selected Axis %u (Node %u / CH%u).\n",
                gCalAxis, gAxes[gCalAxis].nodeId, gAxes[gCalAxis].channel);
  Serial.println("Selected axis is now IDLE for hand positioning.");
  printCalibrationStepPrompt();
  return true;
}

static void captureCalibrationPoint() {
  if (!gCalibrationMode) return;
  if (!gCalAxisSelected) {
    Serial.println("Select axis first: CAL NODE 0, CAL NODE 1, or CAL NODE 2");
    return;
  }
  if (gCalAxis >= 3) return;

  if (!gRt[gCalAxis].measuredValid) {
    Serial.printf("Axis %u has no encoder feedback yet; try again in a moment\n", gCalAxis);
    return;
  }

  float current = gRt[gCalAxis].measuredTurns;

  if (gCalPoint == 0) gCal[gCalAxis].minTurns = current;
  if (gCalPoint == 1) gCal[gCalAxis].maxTurns = current;
  if (gCalPoint == 2) gCal[gCalAxis].homeTurns = current;

  Serial.printf("Captured Axis %u %s = %.4f turns (measured)\n", gCalAxis, calPointName(gCalPoint), current);

  gCalPoint++;
  if (gCalPoint >= 3) {
    gCalPoint = 0;
    gCalComplete = true;

    bool validAxis = isValidAxisCal(gCal[gCalAxis]);
    if (validAxis) {
      Serial.printf("Axis %u calibration captured and valid. Use CAL SAVE to store, or CAL NODE to calibrate another axis.\n", gCalAxis);
    } else {
      Serial.printf("Axis %u capture order is invalid (need MIN < HOME < MAX). Re-run CAL NODE %u and follow direction prompts.\n",
                    gCalAxis, gCalAxis);
    }

    Serial.printf("Axis %u values: MIN=%.4f MAX=%.4f HOME=%.4f\n",
                  gCalAxis,
                  gCal[gCalAxis].minTurns,
                  gCal[gCalAxis].maxTurns,
                  gCal[gCalAxis].homeTurns);
  } else {
    printCalibrationStepPrompt();
  }
}

static void processButton() {
  bool pressed = (digitalRead(CAL_BUTTON_PIN) == LOW);
  unsigned long nowMs = millis();

  if (pressed && !gBtnPrevPressed) {
    gBtnPressStartMs = nowMs;
    gBtnLongHandled = false;
  }

  if (pressed && !gBtnLongHandled && (nowMs - gBtnPressStartMs >= CAL_BUTTON_HOLD_MS)) {
    gBtnLongHandled = true;

    if (!gCalibrationMode) {
      enterCalibrationMode();
    } else {
      bool shouldSave = gCalComplete;
      exitCalibrationMode(shouldSave);
    }
  }

  if (!pressed && gBtnPrevPressed) {
    // Short press used only in calibration mode to capture points.
    if (gCalibrationMode && !gBtnLongHandled) {
      captureCalibrationPoint();
    }
  }

  gBtnPrevPressed = pressed;
}

static bool startsWith(const char* s, const char* prefix) {
  while (*prefix) {
    if (*s != *prefix) {
      return false;
    }
    ++s;
    ++prefix;
  }
  return true;
}

static void toUpperInPlace(char* s) {
  while (*s) {
    if (*s >= 'a' && *s <= 'z') {
      *s = (char)(*s - 'a' + 'A');
    }
    ++s;
  }
}

static char* trimWhitespaceInPlace(char* s) {
  while (*s == ' ' || *s == '\t') {
    ++s;
  }

  char* end = s + strlen(s);
  while (end > s && (end[-1] == ' ' || end[-1] == '\t')) {
    --end;
  }
  *end = '\0';
  return s;
}

static void printCalibrationHelp() {
  Serial.println("Calibration serial commands:");
  Serial.println("  CAL START   -> enter calibration mode");
  Serial.println("  CAL NODE N  -> select axis index 0..2");
  Serial.println("  CAL CAP     -> capture current point (MIN/MAX/HOME, with direction prompts)");
  Serial.println("  CAL SAVE    -> save to FRAM and exit calibration");
  Serial.println("  CAL EXIT    -> exit calibration without saving");
  Serial.println("  CAL STATUS  -> print state + measured turns + vel + POSITIVE/NEGATIVE direction");
  Serial.println("  CAL FEED ON -> enable live turns feed during calibration");
  Serial.println("  CAL FEED OFF-> disable live turns feed during calibration");
}

static void processSerialCalibrationCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      gSerialCmdBuf[gSerialCmdLen] = '\0';
      toUpperInPlace(gSerialCmdBuf);
      char* cmd = trimWhitespaceInPlace(gSerialCmdBuf);

      if (*cmd == '\0') {
        return;
      }

      if (startsWith(cmd, "CAL START")) {
        if (!gCalibrationMode) {
          enterCalibrationMode();
        } else {
          Serial.println("Already in calibration mode");
        }
      } else if (startsWith(cmd, "CAL NODE ")) {
        if (gCalibrationMode) {
          int axis = atoi(cmd + 9);
          selectCalibrationAxis(axis);
        } else {
          Serial.println("Enter calibration mode first: CAL START");
        }
      } else if (startsWith(cmd, "CAL CAP")) {
        if (gCalibrationMode) {
          captureCalibrationPoint();
        } else {
          Serial.println("Not in calibration mode");
        }
      } else if (startsWith(cmd, "CAL SAVE")) {
        if (gCalibrationMode) {
          printSaveReadinessSummary();
          if (gCalComplete) {
            exitCalibrationMode(true);
          } else {
            Serial.println("Capture MIN/MAX/HOME for the selected axis first");
          }
        } else {
          Serial.println("Not in calibration mode");
        }
      } else if (startsWith(cmd, "CAL EXIT")) {
        if (gCalibrationMode) {
          exitCalibrationMode(false);
        } else {
          Serial.println("Not in calibration mode");
        }
      } else if (startsWith(cmd, "CAL STATUS")) {
        Serial.printf("cal_mode=%d selected=%d axis=%u point=%s complete=%d cmd=%.4f meas=%.4f vel=%.4f dir=%s valid=%d\n",
                      gCalibrationMode ? 1 : 0,
                      gCalAxisSelected ? 1 : 0,
                      gCalAxis,
                      calPointName(gCalPoint),
                      gCalComplete ? 1 : 0,
                      gRt[gCalAxis].commandedTurns,
                      gRt[gCalAxis].measuredTurns,
                      gRt[gCalAxis].measuredVel,
                      directionFromVelocity(gRt[gCalAxis].measuredVel),
                      gRt[gCalAxis].measuredValid ? 1 : 0);
      } else if (startsWith(cmd, "CAL FEED ON")) {
        gCalLiveFeed = true;
        gCalLiveLastMs = 0;
        gCalFeedHasLast = false;
        Serial.println("Calibration live feed ON");
      } else if (startsWith(cmd, "CAL FEED OFF")) {
        gCalLiveFeed = false;
        Serial.println("Calibration live feed OFF");
      } else if (startsWith(cmd, "CAL HELP")) {
        printCalibrationHelp();
      } else {
        Serial.print("Unknown cmd: ");
        Serial.println(cmd);
        printCalibrationHelp();
      }

      gSerialCmdLen = 0;
      gSerialCmdBuf[0] = '\0';
      continue;
    }

    if (gSerialCmdLen < sizeof(gSerialCmdBuf) - 1) {
      gSerialCmdBuf[gSerialCmdLen++] = c;
    }
  }
}

void demoArmSetup() {
  Serial.begin(115200);
  delay(200);

  for (int i = 0; i < 16; i++) {
    _ch[i] = TX_CAL_CENTER;
  }
  for (int i = 0; i < 3; i++) {
    gCal[i] = gDefaultCal[i];
    gRt[i].commandedTurns = gCal[i].homeTurns;
    gRt[i].measuredTurns = 0.0f;
    gRt[i].measuredVel = 0.0f;
    gRt[i].measuredValid = false;
    gRt[i].rampStartTurns = gCal[i].homeTurns;
    gRt[i].rampStartMs = 0;
    gRt[i].rampingToHome = false;
  }

  pinMode(CAL_BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(FRAM_I2C_SDA_PIN, FRAM_I2C_SCL_PIN);
  gFramDetected = framProbe();
  gFramSelfTestOk = gFramDetected && framSelfTest();

  bool framLoaded = false;
  if (gFramSelfTestOk) {
    framLoaded = loadCalibrationFromFram();
  }

  CRSF_SERIAL.begin(CRSF_BAUD, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
  _canReady = initCanBus();

  Serial.println("Receiver_Demo_Arm_Fresh_2026 - 3-axis demo arm controller starting");
  Serial.printf("CAN ready: %s\n", _canReady ? "yes" : "no");
  Serial.printf("FRAM detect: %s | self-test: %s\n",
                gFramDetected ? "OK" : "FAIL",
                gFramSelfTestOk ? "OK" : "FAIL");
  Serial.printf("FRAM calibration load: %s\n", framLoaded ? "OK" : "default limits in use");
  Serial.println("CH1->Node0 shoulder | CH2->Node1 upper | CH3->Node2 lower | CH5 arm switch");
  Serial.println("Hold CAL button 5s to enter calibration mode");
  Serial.println("No button yet? Use: CAL START, CAL NODE N, CAL CAP, CAL SAVE, CAL EXIT");
}

static void runNormalControl(unsigned long nowMs) {
  bool armed = hasSignal() && (getChannel(ARM_SWITCH_CHANNEL) > ARM_THRESHOLD);
  processCanRx();

  if (_canReady && armed != _prevArmed) {
    if (armed) {
      for (int i = 0; i < 3; i++) {
        sendOdriveControllerModes(gAxes[i].nodeId, ODRIVE_CONTROL_MODE_POSITION, ODRIVE_INPUT_MODE_PASSTHROUGH);
        sendOdriveSetState(gAxes[i].nodeId, ODRIVE_AXIS_STATE_CLOSED_LOOP);
        gRt[i].rampingToHome = false;
      }
      Serial.println(">>> ARMED - CH control active");
    } else {
      for (int i = 0; i < 3; i++) {
        gRt[i].rampStartTurns = gRt[i].commandedTurns;
        gRt[i].rampStartMs = nowMs;
        gRt[i].rampingToHome = true;
      }
      Serial.println(">>> SAFETY - returning all axes to home");
    }
    _prevArmed = armed;
  }

  if (_canReady && nowMs - _lastOdriveSendMs >= ODRIVE_SEND_PERIOD_MS) {
    _lastOdriveSendMs = nowMs;

    for (int i = 0; i < 3; i++) {
      requestCanFrame(gAxes[i].nodeId, ODRIVE_CMD_GET_ENCODER_ESTIMATES);

      float cmd;
      if (armed) {
        cmd = mapChannelToTurns(getChannel(gAxes[i].channel), gCal[i]);
      } else {
        cmd = getRampToHome(gCal[i], gRt[i], nowMs);
      }

      cmd = clampf(cmd, gCal[i].minTurns, gCal[i].maxTurns);
      gRt[i].commandedTurns = cmd;
      sendOdrivePosition(gAxes[i].nodeId, cmd);
    }
  }

}

static void runCalibrationControl(unsigned long nowMs) {
  processCanRx();

  if (_canReady && nowMs - _lastOdriveSendMs >= ODRIVE_SEND_PERIOD_MS) {
    _lastOdriveSendMs = nowMs;

    for (int i = 0; i < 3; i++) {
      requestCanFrame(gAxes[i].nodeId, ODRIVE_CMD_GET_ENCODER_ESTIMATES);

      // In calibration mode we do not send position commands.
      // This avoids holding torque so joints can be moved by hand.
    }
  }

  printCalibrationLiveFeed(nowMs, false);

}

void demoArmLoop() {
  processCrsf();
  processButton();
  processSerialCalibrationCommands();

  unsigned long nowMs = millis();
  if (gCalibrationMode) {
    runCalibrationControl(nowMs);
  } else {
    runNormalControl(nowMs);
  }
}

uint16_t getChannel(uint8_t ch) {
  if (ch < 1 || ch > 16) return TX_CAL_CENTER;
  return _ch[ch - 1];
}

bool hasSignal() {
  return _hasSignal;
}

bool canBusReady() {
  return _canReady;
}

float getOdrivePositionSetpoint() {
  return gRt[0].commandedTurns;
}

uint8_t getOdriveNodeId() {
  return gAxes[0].nodeId;
}

uint32_t getOdriveCanBaud() {
  return ODRIVE_CAN_BAUD;
}
