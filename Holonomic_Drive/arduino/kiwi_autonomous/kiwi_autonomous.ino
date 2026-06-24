/*
 * Kiwi Drive Teleop Test
 * Hardware:
 *   Arduino Nano
 *   3x PWM ESCs
 *       Motor1 -> D11  (front-right)
 *       Motor2 -> D10  (front-left)
 *       Motor3 -> D9   (rear-centre)
 *   FS2A Receiver (boot-time diagnostics only — see notes below)
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
 * Boot sequence: OLED -> MPU -> ESC arm -> RX check -> motor spin test -> drive loop
 * Serial @ 115200
 *
 * ── Serial protocol (Pi <-> Arduino) ─────────────────────────────────────
 *   Pi -> Arduino : "vx,vy,rx\n"   three floats, roughly in [-1, 1]
 *                    vx/vy = translation command, rx = rotation command
 *                    (the Pi computes rx itself from the yaw below).
 *   Arduino -> Pi  : "yaw\n"       one float per line, degrees [0, 360)
 *                    streamed continuously from the MPU6050.
 *
 *   The drive command has a watchdog: if no valid line arrives from the
 *   Pi for COMMAND_TIMEOUT_MS, motors are stopped rather than continuing
 *   to act on a stale/disconnected command.
 *
 *   NOTE: the old plain-text debug print over Serial has been removed
 *   from the runtime loop, since that link is now reserved for the
 *   vx,vy,rx / yaw protocol above — a stray human-readable line would
 *   just get silently dropped by the Pi's float parser, but it's wasted
 *   bandwidth and bad practice to mix the two on one stream. The OLED
 *   still shows live values for on-robot debugging.
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
unsigned long lastOled  = 0;
const long OLED_MS  = 100;

// ── Drive command / yaw streaming state ──────────────────────────────────────
String serialLineBuf = "";
float cmd_x = 0.0, cmd_y = 0.0, cmd_rx = 0.0;
unsigned long lastCommandMs = 0;
const unsigned long COMMAND_TIMEOUT_MS = 300;  // failsafe: stop if Pi goes quiet
unsigned long lastYawSend = 0;
const long YAW_SEND_MS = 20;                   // ~50Hz yaw stream to the Pi

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

// ── Serial protocol helpers ──────────────────────────────────────────────────

// Non-blocking: consumes whatever bytes are waiting, parses complete
// "vx,vy,rx\n" lines, and updates cmd_x/cmd_y/cmd_rx + the watchdog
// timestamp. Partial lines are left in serialLineBuf for next call.
void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      int firstComma  = serialLineBuf.indexOf(',');
      int secondComma = serialLineBuf.indexOf(',', firstComma + 1);
      if (firstComma > 0 && secondComma > firstComma) {
        cmd_x  = serialLineBuf.substring(0, firstComma).toFloat();
        cmd_y  = serialLineBuf.substring(firstComma + 1, secondComma).toFloat();
        cmd_rx = serialLineBuf.substring(secondComma + 1).toFloat();
        lastCommandMs = millis();
      }
      // Malformed line (e.g. missing a comma) is just dropped; the last
      // good command keeps being used until the watchdog times it out.
      serialLineBuf = "";
    } else if (c != '\r') {
      serialLineBuf += c;
      // Guard against a corrupted/unterminated line growing forever
      if (serialLineBuf.length() > 64) serialLineBuf = "";
    }
  }
}

// Streams yaw back to the Pi at a fixed rate (own line per call - never
// mixed with anything else on Serial during the drive loop).
void sendYaw() {
  unsigned long now = millis();
  if (now - lastYawSend >= YAW_SEND_MS) {
    lastYawSend = now;
    Serial.println(yaw, 2);
  }
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

  // RX check (boot-time diagnostic only — runtime drive comes from the Pi)
  testReceiver();

  // Motor spin test
  motorSpinTest();

  // Ready
  Serial.println(F("[READY] Drive loop starting. Listening for vx,vy,rx..."));
  if (oledOk) { oledClear(); oledLine(F("DRIVE READY")); }
  delay(500);

  lastCommandMs = millis();  // don't immediately trip the watchdog
}

// ─────────────────────────────────────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  if (mpuOk) {
    mpu.update();
    yaw = mpu.getAngleZ();
    while (yaw <   0) yaw += 360;
    while (yaw >= 360) yaw -= 360;
  }

  // Pull in any new vx,vy,rx line(s) from the Pi (non-blocking).
  readSerialCommands();

  unsigned long now = millis();
  bool commandStale = (now - lastCommandMs > COMMAND_TIMEOUT_MS);

  float x  = commandStale ? 0.0 : cmd_x;
  float y  = commandStale ? 0.0 : cmd_y;
  float rx = commandStale ? 0.0 : cmd_rx;

  robotCentricDrive(x, y, rx);

  // Stream yaw back to the Pi at a fixed rate.
  sendYaw();

  // OLED debug (does not touch Serial, so it can't collide with the protocol)
  if (oledOk && now - lastOled >= OLED_MS) {
    lastOled = now;
    oledClear();
    oled.print(F("X:")); oled.print(x, 2);
    oled.print(F(" Y:")); oled.println(y, 2);
    oled.print(F("Rot:")); oled.println(rx, 2);
    if (mpuOk) { oled.print(F("Yaw:")); oled.println(yaw, 1); }
    oled.println(commandStale ? F("LINK: STALE") : F("LINK: OK"));
    oled.display();
  }
}
