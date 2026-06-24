/*
 * Kiwi Drive Teleop Test
 * Hardware:
 *   Arduino Nano
 *   3x PWM ESCs
 *       Motor1 -> D11  (front-right)
 *       Motor2 -> D10  (front-left)
 *       Motor3 -> D9   (rear-centre)
 *   FS2A Receiver
 *       Ch1 (strafe X)   -> D2
 *       Ch2 (fwd/back Y) -> D3
 *       Ch4 (rotation)   -> D4
 *   OLED SSD1306 I2C     -> A4(SDA), A5(SCL)  addr 0x3C
 *   MPU6050              -> A4(SDA), A5(SCL)  addr 0x68
 *
 * Motor layout (viewed from above):
 *
 *        [OLED]
 *       2-------1
 *        \     /
 *         \   /
 *          \ /
 *           3
 *
 * Boot sequence: OLED -> MPU -> ESC arm -> RX check -> motor spin test -> teleop
 * Serial @ 115200
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_light.h>
#include <Servo.h>

// ── Pins ──────────────────────────────────────────────────────────────────────
const int MOTOR1_PIN  = 11;
const int MOTOR2_PIN  = 10;
const int MOTOR3_PIN  = 9;
const int RX_CH1_PIN  = 2;
const int RX_CH2_PIN  = 3;
const int RX_CH4_PIN  = 4;

// ── ESCs ──────────────────────────────────────────────────────────────────────
Servo esc1, esc2, esc3;
const int ESC_NEUTRAL  = 1500;
const int ESC_MAX      = 1900;
const int ESC_MIN      = 1100;
const int ESC_DEADBAND = 30;
const float SPEED_SCALE = 0.5;

// ── OLED ──────────────────────────────────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_ADDR     0x3C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ── IMU ───────────────────────────────────────────────────────────────────────
MPU6050 mpu(Wire);
float yaw = 0;

// ── State ─────────────────────────────────────────────────────────────────────
bool mpuOk  = false;
bool oledOk = false;

// ── Timing ────────────────────────────────────────────────────────────────────
unsigned long lastPrint = 0;
unsigned long lastOled  = 0;
const long PRINT_MS = 200;
const long OLED_MS  = 100;

// ─────────────────────────────────────────────────────────────────────────────
// HELPERS
// ─────────────────────────────────────────────────────────────────────────────

unsigned int readRxChannel(int pin) {
  return pulseIn(pin, HIGH, 25000);
}

float rxToFloat(unsigned int us) {
  if (us == 0) return 0.0;
  int centred = (int)us - 1500;
  if (abs(centred) < ESC_DEADBAND) return 0.0;
  return constrain((float)centred / 500.0, -1.0, 1.0);
}

void writeMotor(Servo &esc, float speed) {
  speed = constrain(speed, -1.0, 1.0);
  int us = ESC_NEUTRAL + (int)(speed * (ESC_MAX - ESC_NEUTRAL) * SPEED_SCALE);
  esc.writeMicroseconds(constrain(us, ESC_MIN, ESC_MAX));
}

void stopAllMotors() {
  esc1.writeMicroseconds(ESC_NEUTRAL);
  esc2.writeMicroseconds(ESC_NEUTRAL);
  esc3.writeMicroseconds(ESC_NEUTRAL);
}

// ── OLED helpers ─────────────────────────────────────────────────────────────
void oledClear() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
}

// Accepts a flash string (F()) to avoid RAM use
void oledLine(const __FlashStringHelper* msg) {
  oled.println(msg);
  oled.display();
}

void oledStatus(const __FlashStringHelper* label, bool ok) {
  oled.print(label);
  oled.print(F(": "));
  oled.println(ok ? F("OK") : F("FAIL"));
  oled.display();
}

// ─────────────────────────────────────────────────────────────────────────────
// TEST ROUTINES
// ─────────────────────────────────────────────────────────────────────────────

bool testOLED() {
  oledClear();
  oledLine(F("OLED: OK"));
  Serial.println(F("[OLED] OK"));
  delay(500);
  return true;
}

bool testMPU() {
  Wire.begin();
  int err = mpu.begin();
  if (err != 0) {
    Serial.print(F("[MPU] FAIL code:"));
    Serial.println(err);
    if (oledOk) { oledClear(); oledLine(F("MPU6050: FAIL")); }
    return false;
  }

  Serial.println(F("[MPU] Calibrating, hold still..."));
  if (oledOk) { oledClear(); oledLine(F("MPU: calibrating\nDo not move!")); }

  mpu.calcGyroOffsets();
  mpu.update();
  yaw = mpu.getAngleZ();

  Serial.print(F("[MPU] OK yaw:"));
  Serial.println(yaw);
  if (oledOk) { oledClear(); oledLine(F("MPU6050: OK")); }
  delay(500);
  return true;
}

bool testReceiver() {
  Serial.println(F("[RX] Checking channels..."));
  if (oledOk) { oledClear(); oledLine(F("RX signal test...")); }

  bool anySignal = false;
  for (int i = 0; i < 3; i++) {
    unsigned int ch1 = readRxChannel(RX_CH1_PIN);
    unsigned int ch2 = readRxChannel(RX_CH2_PIN);
    unsigned int ch4 = readRxChannel(RX_CH4_PIN);

    Serial.print(F("[RX] Ch1="));  Serial.print(ch1);
    Serial.print(F(" Ch2="));      Serial.print(ch2);
    Serial.print(F(" Ch4="));      Serial.print(ch4);
    Serial.println(F("us"));

    if (ch1 > 900 && ch2 > 900 && ch4 > 900) anySignal = true;
    delay(100);
  }

  if (oledOk) { oledClear(); oledStatus(F("RX Ch1/2/4"), anySignal); }
  Serial.print(F("[RX] "));
  Serial.println(anySignal ? F("PASS") : F("FAIL-check TX bound"));
  delay(800);
  return anySignal;
}

void motorSpinTest() {
  Serial.println(F("[MTR] Clear wheels! Send key or wait 3s to skip."));
  if (oledOk) { oledClear(); oledLine(F("Motor test\nKey=run Wait=skip")); }

  unsigned long t = millis();
  bool skip = true;
  while (millis() - t < 3000) {
    if (Serial.available()) { Serial.read(); skip = false; break; }
  }
  if (skip) {
    Serial.println(F("[MTR] Skipped."));
    return;
  }

  const int TEST_SPEED = 1560;
  const int TEST_MS    = 400;

  // Motor 1
  Serial.println(F("[MTR] Motor1 (front-R)"));
  if (oledOk) { oledClear(); oledLine(F("Motor1 front-R")); }
  esc1.writeMicroseconds(TEST_SPEED); delay(TEST_MS);
  esc1.writeMicroseconds(ESC_NEUTRAL); delay(400);

  // Motor 2
  Serial.println(F("[MTR] Motor2 (front-L)"));
  if (oledOk) { oledClear(); oledLine(F("Motor2 front-L")); }
  esc2.writeMicroseconds(TEST_SPEED); delay(TEST_MS);
  esc2.writeMicroseconds(ESC_NEUTRAL); delay(400);

  // Motor 3
  Serial.println(F("[MTR] Motor3 (rear-C)"));
  if (oledOk) { oledClear(); oledLine(F("Motor3 rear-C")); }
  esc3.writeMicroseconds(TEST_SPEED); delay(TEST_MS);
  esc3.writeMicroseconds(ESC_NEUTRAL); delay(400);

  Serial.println(F("[MTR] Done."));
  if (oledOk) { oledClear(); oledLine(F("Motor test done")); }
  delay(500);
}

// ─────────────────────────────────────────────────────────────────────────────
// KIWI DRIVE  — Robot Centric
// ─────────────────────────────────────────────────────────────────────────────
void robotCentricDrive(float x, float y, float rx) {
  float m1 = (-0.5f * x + (sqrt(3.0) / 2.0f) * y) - rx * 0.5f;
  float m2 = -(-0.5f * x - (sqrt(3.0) / 2.0f) * y - rx * 0.5f);
  float m3 = x - rx * 0.5f;

  float maxVal = max(max(max(abs(m1), abs(m2)), abs(m3)), 1.0f);
  m1 /= maxVal;
  m2 /= maxVal;
  m3 /= maxVal;

  writeMotor(esc1, m1);
  writeMotor(esc2, m2);
  writeMotor(esc3, m3);
}

// ─────────────────────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println(F("\n=== Kiwi Teleop Test ==="));

  pinMode(RX_CH1_PIN, INPUT);
  pinMode(RX_CH2_PIN, INPUT);
  pinMode(RX_CH4_PIN, INPUT);

  // OLED
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (!oledOk) Serial.println(F("[OLED] FAIL-check wiring"));
  else         testOLED();

  // MPU
  mpuOk = testMPU();

  // ESC arm
  Serial.println(F("[ESC] Arming at neutral..."));
  if (oledOk) { oledClear(); oledLine(F("Arming ESCs...")); }
  esc1.attach(MOTOR1_PIN, ESC_MIN, ESC_MAX);
  esc2.attach(MOTOR2_PIN, ESC_MIN, ESC_MAX);
  esc3.attach(MOTOR3_PIN, ESC_MIN, ESC_MAX);
  stopAllMotors();
  delay(3000);
  Serial.println(F("[ESC] Armed."));
  if (oledOk) { oledClear(); oledLine(F("ESCs armed")); }
  delay(500);

  // RX check
  testReceiver();

  // Motor spin test
  motorSpinTest();

  // Ready
  Serial.println(F("[READY] Teleop loop starting."));
  if (oledOk) { oledClear(); oledLine(F("TELEOP READY")); }
  delay(500);
}

// ─────────────────────────────────────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  unsigned int ch1Raw = readRxChannel(RX_CH1_PIN);
  unsigned int ch2Raw = readRxChannel(RX_CH2_PIN);
  unsigned int ch4Raw = readRxChannel(RX_CH4_PIN);

  float x  = rxToFloat(ch1Raw);
  float y  = rxToFloat(ch2Raw);
  float rx = rxToFloat(ch4Raw);

  if (mpuOk) {
    mpu.update();
    yaw = mpu.getAngleZ();
    while (yaw <   0) yaw += 360;
    while (yaw >= 360) yaw -= 360;
  }

  robotCentricDrive(x, y, rx);

  unsigned long now = millis();

  // Serial debug
  if (now - lastPrint >= PRINT_MS) {
    lastPrint = now;
    Serial.print(F("C1=")); Serial.print(ch1Raw);
    Serial.print(F(" C2=")); Serial.print(ch2Raw);
    Serial.print(F(" C4=")); Serial.print(ch4Raw);
    Serial.print(F(" x=")); Serial.print(x, 2);
    Serial.print(F(" y=")); Serial.print(y, 2);
    Serial.print(F(" rx=")); Serial.print(rx, 2);
    if (mpuOk) { Serial.print(F(" yaw=")); Serial.print(yaw, 1); }
    Serial.println();
  }

  // OLED
  if (oledOk && now - lastOled >= OLED_MS) {
    lastOled = now;
    oledClear();
    oled.print(F("X:")); oled.print(x, 2);
    oled.print(F(" Y:")); oled.println(y, 2);
    oled.print(F("Rot:")); oled.println(rx, 2);
    if (mpuOk) { oled.print(F("Yaw:")); oled.println(yaw, 1); }
    oled.print(F("C1:")); oled.print(ch1Raw);
    oled.print(F(" C2:")); oled.println(ch2Raw);
    oled.print(F("C4:")); oled.println(ch4Raw);
    oled.display();
  }
}