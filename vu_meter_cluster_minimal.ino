#include <SPI.h>
#include <mcp_can.h>

const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);

// ---------------------------------------------------------------------
// Real VW MQB CRC-8 lookup table (P_L_CC_CRC_LUT_APV)
// ---------------------------------------------------------------------
unsigned char CRC_LUT[256] = {
  0, 47, 94, 113, 188, 147, 226, 205, 87, 120, 9, 38, 235, 196, 181, 154, 174, 129, 240, 223, 18, 61, 76, 99, 249, 214, 167, 136, 69, 106,
  27, 52, 115, 92, 45, 2, 207, 224, 145, 190, 36, 11, 122, 85, 152, 183, 198, 233, 221, 242, 131, 172, 97, 78, 63, 16, 138, 165, 212, 251, 54, 25, 104, 71, 230, 201, 184, 151, 90, 117,
  4, 43, 177, 158, 239, 192, 13, 34, 83, 124, 72, 103, 22, 57, 244, 219, 170, 133, 31, 48, 65, 110, 163, 140, 253, 210, 149, 186, 203, 228, 41, 6, 119, 88, 194, 237, 156, 179, 126, 81,
  32, 15, 59, 20, 101, 74, 135, 168, 217, 246, 108, 67, 50, 29, 208, 255, 142, 161, 227, 204, 189, 146, 95, 112, 1, 46, 180, 155, 234, 197, 8, 39, 86, 121, 77, 98, 19, 60, 241, 222,
  175, 128, 26, 53, 68, 107, 166, 137, 248, 215, 144, 191, 206, 225, 44, 3, 114, 93, 199, 232, 153, 182, 123, 84, 37, 10, 62, 17, 96, 79, 130, 173, 220, 243, 105, 70, 55, 24, 213, 250,
  139, 164, 5, 42, 91, 116, 185, 150, 231, 200, 82, 125, 12, 35, 238, 193, 176, 159, 171, 132, 245, 218, 23, 56, 73, 102, 252, 211, 162, 141, 64, 111, 30, 49, 118, 89, 40, 7, 202, 229,
  148, 187, 33, 14, 127, 80, 157, 178, 195, 236, 216, 247, 134, 169, 100, 75, 58, 21, 143, 160, 209, 254, 51, 28, 109, 66
};

// Seed Tables
unsigned char KENNUNG_KlemmenStatus01[16] = { 0xc3,0xc3,0xc3,0xc3,0xc3,0xc3,0xc3,0xc3,0xc3,0xc3,0xc3,0xc3,0xc3,0xc3,0xc3,0xc3 };
unsigned char KENNUNG_MCODE01[16]         = { 0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47,0x47 };
unsigned char KENNUNG_ESP21[16]           = { 0xb4,0xef,0xf8,0x49,0x1e,0xe5,0xc2,0xc0,0x97,0x19,0x3c,0xc9,0xf1,0x98,0xd6,0x61 };
unsigned char KENNUNG_ESP24[16]           = { 0x67,0x8A,0xAE,0x22,0x4D,0xD0,0x51,0x80,0x5C,0xB9,0xCE,0x1E,0xDF,0x02,0x2D,0xD4 };
unsigned char KENNUNG_ESP20[16]           = { 0xAC,0xB3,0xAB,0xEB,0x7A,0xE1,0x3B,0xF7,0x73,0xBA,0x7C,0x9E,0x06,0x5F,0x02,0xD9 };
unsigned char KENNUNG_TSK07[16]           = { 0x78,0x68,0x3A,0x31,0x16,0x08,0x4F,0xDE,0xF7,0x35,0x19,0xE6,0x28,0x2F,0x59,0x82 };
unsigned char KENNUNG_LH_EPS01[16]        = { 0x29,0x29,0x29,0x29,0x29,0x29,0x29,0x29,0x29,0x29,0x29,0x29,0x29,0x29,0x29,0x29 };
unsigned char KENNUNG_WBA03[16]           = { 0x47,0x94,0x92,0x6A,0x67,0xB5,0x0D,0x38,0xE3,0x8A,0x5D,0xB4,0x54,0xAB,0xAE,0x27 };
unsigned char KENNUNG_AIRBAG01[16]        = { 0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40 };
unsigned char KENNUNG_LICHT_ANF[16]       = { 0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07 };

// IDs
#define KLEMMEN_STATUS_01_ID 0x3C0
#define MOTOR_CODE_01_ID     0x641
#define MOTOR_04_ID          0x107
#define ESP_21_ID            0xFD
#define ESP_24_ID            0x31B
#define MOTOR_14_ID          0x3BE
#define GATEWAY_76_ID        0x3df
#define AIRBAG_01_ID         0x40
#define ESP_20_ID            0x65D
#define TSK_07_ID            0x31E
#define LH_EPS_01_ID         0x32A
#define RKA_01_ID            0x663
#define WBA_03_ID            0x394
#define BLINKMODI_02_ID      0x366
#define TPMS_ID              0x64A
#define PARKBRAKE_ID         0x30d
#define LICHT_VORNE_01_ID    0x658
#define LICHT_HINTEN_01_ID   0x3D6
#define LICHT_ANF_ID         0x3D5
#define DOOR_STATUS_ID       0x583
#define OUTDOOR_TEMP_ID      0x5e1

// Buffers
unsigned char kStatusBuf[4]      = { 0x00, 0x00, 0x03, 0x00 };
unsigned char motorCode01Buf[8]  = { 0x00, 0x00, 0x00, 0xE8, 0x03, 0x00, 0x00, 0x00 };
unsigned char motor04Buf[8]      = { 0x00, 0x00, 0x00, 0xE8, 0x03, 0x00, 0x00, 0x00 };
unsigned char esp21Buf[8]        = { 0x00, 0xD0, 0x1F, 0x80, 0xd8, 0x0d, 0x00, 0x00 };
unsigned char esp24Buf[8]        = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00 };
unsigned char motor14Buf[8]      = { 0x00, 0x30, 0xE7, 0x51, 0x88, 0xC0, 0x0C, 0x00 };
unsigned char gateway76Buf[8]    = { 0x04, 0x91, 0x00, 0x28, 0x15, 0x00, 0x00, 0x00 };
unsigned char airbag01Buf[8]     = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char esp20Buf[8]        = { 0x00, 0x30, 0x2B, 0x12, 0x00, 0x00, 0xB4, 0x79 };
unsigned char tsk07Buf[8]        = { 0xCA, 0xEF, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x40 };
unsigned char lhEps01Buf[8]      = { 0x4B, 0x08, 0x00, 0x00, 0x02, 0x02, 0x00, 0x00 };
unsigned char rka01Buf[8]        = { 0x60, 0x28, 0x20, 0x07, 0x0D, 0xBC, 0x2C, 0x00 };
unsigned char wba03Buf[8]        = { 0x29, 0x40, 0x00, 0x04, 0x00, 0x04, 0x00, 0x00 };
unsigned char blinkerBuff[8]     = { 0x00, 0x00, 0x04, 0x14, 0x0A, 0xFC, 0x00, 0x00 };
unsigned char tpmsBuff[8]        = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char parkBrakeBuff[4]   = { 0x00, 0x00, 0x00, 0x00 };
unsigned char lichtVorne01Buff[8]= { 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char doorStatusBuff[8]  = { 0x00, 0x10, 0x05, 0x00, 0x00, 0x44, 0x55, 0x00 };
unsigned char outdoorTempBuff[8] = { 0x9A, 0x2A, 0x00, 0x60, 0xFE, 0x00, 0x00, 0x00 };
unsigned char lichtAnfBuff[8]    = { 0x00, 0x04, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char lichtHintenBuff[8] = { 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

unsigned long vSpeedGlobal = 0;
double accumulatedDistanceUnits = 0;
unsigned char seq = 0;

void sendIgnitionStatus(bool ignition) {
  kStatusBuf[2] = ignition ? 0x03 : 0x01;
  unsigned char crc = seq ^ 0xFF;
  for (unsigned char i = 2; i <= 3; i++) {
    crc = CRC_LUT[crc];
    crc = kStatusBuf[i] ^ crc;
  }
  crc = CRC_LUT[crc] ^ KENNUNG_KlemmenStatus01[seq];
  crc = CRC_LUT[crc];
  crc = (~crc);
  kStatusBuf[0] = crc;
  kStatusBuf[1] = seq;
  CAN.sendMsgBuf(KLEMMEN_STATUS_01_ID, 0, 4, kStatusBuf);
}

// CHANGED: now takes tempVal too. Only new line is the motor04Buf[2] write —
// byte 0 and byte 1 of motor04Buf are untouched, exactly as in your working
// version (they were never CRC'd/counted before, and RPM worked anyway, so
// left alone here).
void sendMotor(int rpm, int tempVal) {
  unsigned char crc = (0x10 | seq) ^ 0xFF;
  for (unsigned char i = 2; i <= 7; i++) {
    crc = CRC_LUT[crc];
    crc = motorCode01Buf[i] ^ crc;
  }
  crc = CRC_LUT[crc] ^ KENNUNG_MCODE01[seq];
  crc = CRC_LUT[crc];
  crc = (~crc);

  motorCode01Buf[0] = crc;
  motorCode01Buf[1] = 0x10 | seq;
  CAN.sendMsgBuf(MOTOR_CODE_01_ID, 0, 8, motorCode01Buf);

  unsigned long rpmVal = (unsigned long)rpm / 3;
  motor04Buf[3] = rpmVal % 256;
  motor04Buf[4] = rpmVal / 256;

  // NEW: coolant temp needle driven by bass. Offset formula: raw byte = °C + 48
  motor04Buf[2] = (unsigned char)(tempVal + 48);

  CAN.sendMsgBuf(MOTOR_04_ID, 0, 8, motor04Buf);
}

void sendSpeed(int speedKmh) {
  unsigned long vSpeed = (unsigned long)speedKmh * 98.5;
  vSpeedGlobal = vSpeed;

  esp21Buf[4] = vSpeed % 256;
  esp21Buf[5] = vSpeed / 256;

  unsigned char crc = (0xD0 | seq) ^ 0xFF;
  for (unsigned char i = 2; i <= 7; i++) {
    crc = CRC_LUT[crc];
    crc = esp21Buf[i] ^ crc;
  }
  crc = CRC_LUT[crc] ^ KENNUNG_ESP21[seq];
  crc = CRC_LUT[crc];
  crc = (~crc);

  esp21Buf[0] = crc;
  esp21Buf[1] = 0xD0 | seq;

  CAN.sendMsgBuf(ESP_21_ID, 0, 8, esp21Buf);
  CAN.sendMsgBuf(MOTOR_14_ID, 0, 8, motor14Buf);
  CAN.sendMsgBuf(GATEWAY_76_ID, 0, 8, gateway76Buf);
}

void sendESP24() {
  unsigned long esp24Speed = vSpeedGlobal * 1.35;
  esp24Buf[2] = esp24Speed % 256;
  esp24Buf[3] = esp24Speed / 256;

  double speed_mph = (double)vSpeedGlobal / 100.0 * 0.621371;
  double time_interval_hours = 0.05 / 3600.0;
  double distance_increment_miles = speed_mph * time_interval_hours;
  double distance_increment_units = distance_increment_miles * (7195.0 / 0.1);
  accumulatedDistanceUnits += distance_increment_units;

  uint16_t distance_for_cluster = (uint16_t)accumulatedDistanceUnits;
  esp24Buf[5] = distance_for_cluster & 0xFF;
  esp24Buf[6] = (distance_for_cluster >> 8) & 0xFF;

  if (vSpeedGlobal == 0 || accumulatedDistanceUnits >= 30000) {
    accumulatedDistanceUnits = 0;
  }

  unsigned char crc = seq ^ 0xFF;
  for (unsigned char i = 2; i <= 7; i++) {
    crc = CRC_LUT[crc];
    crc = esp24Buf[i] ^ crc;
  }
  crc = CRC_LUT[crc] ^ KENNUNG_ESP24[seq];
  crc = CRC_LUT[crc];
  crc = (~crc);

  esp24Buf[0] = crc;
  esp24Buf[1] = seq;

  CAN.sendMsgBuf(ESP_24_ID, 0, 8, esp24Buf);
}

void sendAirbag01() {
  unsigned char crc = seq ^ 0xFF;
  for (unsigned char i = 2; i <= 7; i++) {
    crc = CRC_LUT[crc];
    crc = airbag01Buf[i] ^ crc;
  }
  crc = CRC_LUT[crc] ^ KENNUNG_AIRBAG01[seq];
  crc = CRC_LUT[crc];
  crc = (~crc);

  airbag01Buf[0] = crc;
  airbag01Buf[1] = seq;
  CAN.sendMsgBuf(AIRBAG_01_ID, 0, 8, airbag01Buf);
}

void sendESP20() {
  unsigned char crc = (0x30 | seq) ^ 0xFF;
  for (unsigned char i = 2; i <= 7; i++) {
    crc = CRC_LUT[crc];
    crc = esp20Buf[i] ^ crc;
  }
  crc = CRC_LUT[crc] ^ KENNUNG_ESP20[seq];
  crc = CRC_LUT[crc];
  crc = (~crc);

  esp20Buf[0] = crc;
  esp20Buf[1] = 0x30 | seq;
  CAN.sendMsgBuf(ESP_20_ID, 0, 8, esp20Buf);
}

void sendTSK07() {
  unsigned char crc = (0xE0 | seq) ^ 0xFF;
  for (unsigned char i = 2; i <= 7; i++) {
    crc = CRC_LUT[crc];
    crc = tsk07Buf[i] ^ crc;
  }
  crc = CRC_LUT[crc] ^ KENNUNG_TSK07[seq];
  crc = CRC_LUT[crc];
  crc = (~crc);

  tsk07Buf[0] = crc;
  tsk07Buf[1] = 0xE0 | seq;
  CAN.sendMsgBuf(TSK_07_ID, 0, 8, tsk07Buf);
}

void sendLhEPS01() {
  unsigned char crc = seq ^ 0xFF;
  for (unsigned char i = 2; i <= 7; i++) {
    crc = CRC_LUT[crc];
    crc = lhEps01Buf[i] ^ crc;
  }
  crc = CRC_LUT[crc] ^ KENNUNG_LH_EPS01[seq];
  crc = CRC_LUT[crc];
  crc = (~crc);

  lhEps01Buf[0] = crc;
  lhEps01Buf[1] = seq;
  CAN.sendMsgBuf(LH_EPS_01_ID, 0, 8, lhEps01Buf);
}

void sendGear(uint8_t gear) {
  uint8_t tempGear = 0;
  uint8_t tempGearSelector = 0;
  switch (gear) {
    case 10: tempGearSelector = 0x10; break; // P
    case 11: tempGearSelector = 0x20; break; // R
    case 12: tempGearSelector = 0x30; break; // N
    case 13: tempGearSelector = 0x40; break; // D
  }

  wba03Buf[1] = tempGearSelector | seq;
  wba03Buf[3] = tempGear;

  unsigned char crc = ((wba03Buf[1] & 0xF0) | seq) ^ 0xFF;
  for (unsigned char i = 2; i <= 7; i++) {
    crc = CRC_LUT[crc];
    crc = wba03Buf[i] ^ crc;
  }
  crc = CRC_LUT[crc] ^ KENNUNG_WBA03[seq];
  crc = CRC_LUT[crc];
  crc = (~crc);

  wba03Buf[0] = crc;
  wba03Buf[1] = (wba03Buf[1] & 0xF0) | seq;

  CAN.sendMsgBuf(WBA_03_ID, 0, 8, wba03Buf);
  CAN.sendMsgBuf(RKA_01_ID, 0, 8, rka01Buf);
}

void sendBlinkersOff() {
  blinkerBuff[1] = seq;
  blinkerBuff[3] = 0;
  CAN.sendMsgBuf(BLINKMODI_02_ID, 0, 8, blinkerBuff);
}

void sendParkBrakeOff() {
  parkBrakeBuff[0] = 0x00;
  CAN.sendMsgBuf(PARKBRAKE_ID, 0, 4, parkBrakeBuff);
}

void sendTPMSOk() { CAN.sendMsgBuf(TPMS_ID, 0, 8, tpmsBuff); }

void sendLightsOff() {
  lichtVorne01Buff[1] = 0x00;
  lichtVorne01Buff[2] = 0x04;
  CAN.sendMsgBuf(LICHT_VORNE_01_ID, 0, 8, lichtVorne01Buff);
}

void sendOtherLightsOk() {
  unsigned char crc = (0xC0 | seq) ^ 0xFF;
  for (unsigned char i = 2; i <= 7; i++) {
    crc = CRC_LUT[crc];
    crc = lichtAnfBuff[i] ^ crc;
  }
  crc = CRC_LUT[crc] ^ KENNUNG_LICHT_ANF[seq];
  crc = CRC_LUT[crc];
  crc = (~crc);

  lichtAnfBuff[0] = crc;
  lichtAnfBuff[1] = 0xC0 | seq;
  CAN.sendMsgBuf(LICHT_ANF_ID, 0, 8, lichtAnfBuff);

  lichtHintenBuff[0] = seq;
  CAN.sendMsgBuf(LICHT_HINTEN_01_ID, 0, 8, lichtHintenBuff);
}

void sendDoorsClosed() {
  doorStatusBuff[3] = 0;
  CAN.sendMsgBuf(DOOR_STATUS_ID, 0, 8, doorStatusBuff);
}

void sendOutdoorTemperature(int temperature) {
  outdoorTempBuff[0] = (50 + temperature) << 1;
  CAN.sendMsgBuf(OUTDOOR_TEMP_ID, 0, 8, outdoorTempBuff);
}

// ---------------------------------------------------------------------
// Timing & VU Meter State Variables — CYCLE_MS left at 50 (unchanged)
// ---------------------------------------------------------------------
const unsigned long CYCLE_MS = 50;
const unsigned long SLOW_CYCLE_MS = 500;
unsigned long lastSend = 0;
unsigned long lastSlowSend = 0;
unsigned long lastSerialTime = 0;

int targetRpm = 0;
int targetSpeedKmh = 0;
int targetTempC = 50; // NEW: minimum gauge value (50°C)

// Sweep fallback variables
int sweepRpm = 0;
int sweepSpeed = 0;
int sweepTemp = 50; // NEW
bool sweepingUp = true;

const int IDLE_TEMP_C = 90; // typical resting coolant temp once startup animation ends
void runStartupAnimation(); // forward declaration

void setup() {
  Serial.begin(115200);
  unsigned long serialWait = millis();
  while (!Serial && millis() - serialWait < 2000) {}

  bool ok = false;
  for (int attempt = 0; attempt < 20 && !ok; attempt++) {
    if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
      ok = true;
    } else {
      delay(100);
    }
  }
  if (!ok) {
    while (1) {}
  }

  CAN.setMode(MCP_NORMAL);
  Serial.println("CAN Active - Ready for Serial VU Audio!");

  runStartupAnimation();

  // Settle into resting/idle state after the sweep, before normal
  // audio-driven operation takes over
  targetRpm = 800;
  targetSpeedKmh = 0;
  targetTempC = IDLE_TEMP_C;
}

// ---------------------------------------------------------------------
// Power-on self-test sweep — mimics the real cluster's ignition-on
// needle sweep. Blocking is fine here: nothing else needs to run
// before normal loop() operation begins.
// ---------------------------------------------------------------------
void runStartupAnimation() {
  const int STEPS = 40;
  const int STEP_DELAY_MS = 25; // ~1s sweep up, ~1s sweep down = ~2s total

  for (int pass = 0; pass < 2; pass++) {
    bool ascending = (pass == 0);
    for (int i = 0; i <= STEPS; i++) {
      int step = ascending ? i : (STEPS - i);
      int rpm  = map(step, 0, STEPS, 800, 7500);
      int spd  = map(step, 0, STEPS, 0, 240);
      int temp = map(step, 0, STEPS, 50, 130);

      sendIgnitionStatus(true);
      sendMotor(rpm, temp);
      sendSpeed(spd);
      sendESP24();
      sendESP20();
      sendTSK07();
      sendLhEPS01();
      sendGear(10);
      sendAirbag01();
      sendBlinkersOff();
      sendParkBrakeOff();

      seq++;
      if (seq > 15) seq = 0;
      delay(STEP_DELAY_MS);
    }
  }
}

void loop() {
  unsigned long now = millis();

  // 1. READ SERIAL AUDIO DATA — CHANGED: "L,R\n" -> "L,R,BASS\n"
  // Parsing style (readStringUntil + indexOf) is unchanged from your
  // original, just extended to a third field.
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    int firstComma  = input.indexOf(',');
    int secondComma = input.indexOf(',', firstComma + 1);

    if (firstComma != -1 && secondComma != -1) {
      int audioLeft  = input.substring(0, firstComma).toInt();
      int audioRight = input.substring(firstComma + 1, secondComma).toInt();
      int audioBass  = input.substring(secondComma + 1).toInt();

      targetRpm      = map(constrain(audioLeft, 0, 255), 0, 255, 800, 7500);
      targetSpeedKmh = map(constrain(audioRight, 0, 255), 0, 255, 0, 240);
      // Bass field is still parsed (Python still sends it) but no longer
      // drives the temp gauge — temp stays at IDLE_TEMP_C after the
      // startup animation. audioBass intentionally unused here.
      (void)audioBass;

      lastSerialTime = now;
    }
  }

  // 2. 50ms CAN BUS UPDATE LOOP — unchanged cadence
  if (now - lastSend >= CYCLE_MS) {
    lastSend = now;

    if (now - lastSerialTime > 2000) {
      if (sweepingUp) {
        sweepRpm += 80;
        sweepSpeed += 3;
        sweepTemp += 1; // NEW
        if (sweepRpm >= 7000) sweepingUp = false;
      } else {
        sweepRpm -= 80;
        sweepSpeed -= 3;
        sweepTemp -= 1; // NEW
        if (sweepRpm <= 0) sweepingUp = true;
      }
      targetRpm = sweepRpm;
      targetSpeedKmh = sweepSpeed;
      targetTempC = constrain(sweepTemp, 50, 130); // NEW
    }

    sendIgnitionStatus(true);
    sendMotor(targetRpm, targetTempC); // CHANGED: now passes tempVal
    sendSpeed(targetSpeedKmh);
    sendESP24();
    sendESP20();
    sendTSK07();
    sendLhEPS01();
    sendGear(10); // Park
    sendAirbag01();
    sendBlinkersOff();
    sendParkBrakeOff();

    if (now - lastSlowSend >= SLOW_CYCLE_MS) {
      lastSlowSend = now;
      sendTPMSOk();
      sendLightsOff();
      sendOtherLightsOk();
      sendDoorsClosed();
      sendOutdoorTemperature(20);
    }

    seq++;
    if (seq > 15) seq = 0;
  }
}
