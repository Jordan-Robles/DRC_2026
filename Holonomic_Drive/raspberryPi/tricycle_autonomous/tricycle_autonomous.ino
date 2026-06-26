/*
 * Front-Drive / Rear-Steer Autonomous Test
 * (Refactored from the original Kiwi Drive sketch -- same boot sequence,
 *  watchdog, MPU yaw streaming, and OLED debug. Only the actuation layer
 *  changed: 3 omni-wheel ESCs -> 2 drive ESCs + 1 steering servo.)
 *
 * Hardware:
 *   Arduino Nano
 *   2x PWM ESCs (front drive wheels, both spin together -- no differential)
 *       Motor1 -> D11  (front-right)
 *       Motor2 -> D10  (front-left)
 *   1x hobby servo (rear steering wheel)
 *       Steering -> D9
 *   OLED SSD1306 I2C     -> A4(SDA), A5(SCL)  addr 0x3C
 *   MPU6050               -> A4(SDA), A5(SCL)  addr 0x68
 *
 * CALIBRATE before trusting this on the track:
 *   STEER_CENTER_US / STEER_MIN_US / STEER_MAX_US -- depends entirely on
 *   your specific servo and steering linkage. Center the linkage by hand,
 *   measure what pulse width your servo needs for that position, and set
 *   STEER_CENTER_US to that. Then find how far it can turn each way before
 *   binding and set MIN/MAX accordingly.
 *
 * Boot sequence: OLED -> MPU -> ESC arm + servo centre -> drive loop
 * (the old FS2A receiver boot-time diagnostic step has been removed --
 *  there's no RX wiring on this chassis. If you still have the receiver
 *  wired up for manual override, that can be re-added the same way it was
 *  in the kiwi sketch.)
 * Serial @ 115200
 *
 * ── Serial protocol (Pi <-> Arduino) ─────────────────────────────────────
 *   Pi -> Arduino : "drive,steer\n"   two floats
 *                    drive = front-wheel speed, roughly [-1, 1]
 *                    steer = rear-wheel angle in DEGREES, 0 = straight,
 *                            positive = turn right
 *                    (see pi_bridge.py / drivetrain.py on the Pi side)
 *   Arduino -> Pi  : "yaw\n"          one float per line, degrees [0, 360)
 *                    streamed continuously from the MPU6050.
 *
 *   Same watchdog as before: if no valid line arrives from the Pi for
 *   COMMAND_TIMEOUT_MS, motors stop and steering centres rather than
 *   continuing to act on a stale/disconnected command.
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_light.h>
#include <Servo.h>

// ── Pins ──────────────────────────────────────────────────────────────────────
const int MOTOR1_PIN  = 11;   // front-right drive ESC
const int MOTOR2_PIN  = 10;   // front-left drive ESC
const int STEER_PIN   = 9;    // rear steering servo

// ── Drive ESCs ────────────────────────────────────────────────────────────────
Servo esc1, esc2;
const int ESC_NEUTRAL  = 1500;
const int ESC_MAX      = 1900;
const int ESC_MIN      = 1100;
const float SPEED_SCALE = 1;

// ── Steering servo ────────────────────────────────────────────────────────────
Servo steerServo;
// CALIBRATE: these three depend on your servo + linkage. STEER_CENTER_US is
// "wheel pointed straight ahead". The MIN/MAX should match the mechanical
// limits of your steering linkage, not just the servo's own full range.
int STEER_CENTER_US = 1500;
int STEER_MIN_US    = 1100; //1100
int STEER_MAX_US    = 1900; //1900
// Degrees of mechanical travel corresponding to (STEER_MAX_US - STEER_CENTER_US).
// Must match drivetrain.py's MAX_STEER_DEG on the Pi side, or the mapping
// below will under- or over-rotate relative to what the Pi thinks it asked
// for. CALIBRATE alongside MAX_STEER_DEG in drivetrain.py.
const float STEER_MAX_DEG = 30.0;

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
float cmd_drive = 0.0, cmd_steer_deg = 0.0;
unsigned long lastCommandMs = 0;
const unsigned long COMMAND_TIMEOUT_MS = 300;  // failsafe: stop if Pi goes quiet
unsigned long lastYawSend = 0;
const long YAW_SEND_MS = 20;                   // ~50Hz yaw stream to the Pi

// ─────────────────────────────────────────────────────────────────────────────
// HELPERS
// ─────────────────────────────────────────────────────────────────────────────

void writeMotor(Servo &esc, float speed) {
  speed = constrain(speed, -1.0, 1.0);
  int us = ESC_NEUTRAL + (int)(speed * (ESC_MAX - ESC_NEUTRAL) * SPEED_SCALE);
  esc.writeMicroseconds(constrain(us, ESC_MIN, ESC_MAX));
}

void writeSteer(float steer_deg) {
  steer_deg = constrain(steer_deg, -STEER_MAX_DEG, STEER_MAX_DEG);
  int us;
  if (steer_deg >= 0) {
    us = STEER_CENTER_US + (int)((steer_deg / STEER_MAX_DEG) * (STEER_MAX_US - STEER_CENTER_US));
  } else {
    us = STEER_CENTER_US + (int)((steer_deg / STEER_MAX_DEG) * (STEER_CENTER_US - STEER_MIN_US));
  }
  steerServo.writeMicroseconds(constrain(us, STEER_MIN_US, STEER_MAX_US));
}

void stopAllMotors() {
  esc1.writeMicroseconds(ESC_NEUTRAL);
  esc2.writeMicroseconds(ESC_NEUTRAL);
  steerServo.writeMicroseconds(STEER_CENTER_US);
}

// ── Serial protocol helpers ──────────────────────────────────────────────────

// Non-blocking: consumes whatever bytes are waiting, parses complete
// "drive,steer\n" lines, and updates cmd_drive/cmd_steer_deg + the watchdog
// timestamp. Partial lines are left in serialLineBuf for next call.
void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      int comma = serialLineBuf.indexOf(',');
      if (comma > 0) {
        cmd_drive     = serialLineBuf.substring(0, comma).toFloat();
        cmd_steer_deg = serialLineBuf.substring(comma + 1).toFloat();
        lastCommandMs = millis();
      }
      // Malformed line (e.g. missing the comma) is just dropped; the last
      // good command keeps being used until the watchdog times it out.
      serialLineBuf = "";
    } else if (c != '\r') {
      serialLineBuf += c;
      // Guard against a corrupted/unterminated line growing forever
      if (serialLineBuf.length() > 64) serialLineBuf = "";
    }
  }
}

// Streams yaw back to the Pi at a fixed rate (own line per call -- never
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

// ─────────────────────────────────────────────────────────────────────────────
// FRONT-DRIVE / REAR-STEER  -- Robot Centric
// ─────────────────────────────────────────────────────────────────────────────
// No differential -- both front wheels are pure drive wheels in this
// layout, so they always get the same speed. Direction comes entirely from
// the rear steering wheel.
void tricycleDrive(float drive, float steer_deg) {
  writeMotor(esc1, drive);
  writeMotor(esc2, -drive);
  writeSteer(steer_deg);
}

// ─────────────────────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println(F("\n=== Front-Drive/Rear-Steer Test ==="));

  // OLED
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (!oledOk) Serial.println(F("[OLED] FAIL-check wiring"));
  else         testOLED();

  // MPU
  mpuOk = testMPU();

  // ESC arm + steering centre
  Serial.println(F("[ESC] Arming at neutral, centring steering..."));
  if (oledOk) { oledClear(); oledLine(F("Arming ESCs...")); }
  esc1.attach(MOTOR1_PIN, ESC_MIN, ESC_MAX);
  esc2.attach(MOTOR2_PIN, ESC_MIN, ESC_MAX);
  steerServo.attach(STEER_PIN, STEER_MIN_US, STEER_MAX_US);
  stopAllMotors();
  delay(3000);
  Serial.println(F("[ESC] Armed."));
  if (oledOk) { oledClear(); oledLine(F("ESCs armed")); }
  delay(500);

  // Ready
  Serial.println(F("[READY] Drive loop starting. Listening for drive,steer..."));
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

  // Pull in any new drive,steer line(s) from the Pi (non-blocking).
  readSerialCommands();

  unsigned long now = millis();
  bool commandStale = (now - lastCommandMs > COMMAND_TIMEOUT_MS);

  float drive = commandStale ? 0.0 : cmd_drive;
  float steer = commandStale ? 0.0 : cmd_steer_deg;

  tricycleDrive(drive, steer);

  // Stream yaw back to the Pi at a fixed rate.
  sendYaw();

  // OLED debug (does not touch Serial, so it can't collide with the protocol)
  if (oledOk && now - lastOled >= OLED_MS) {
    lastOled = now;
    oledClear();
    oled.print(F("Drive:")); oled.println(drive, 2);
    oled.print(F("Steer:")); oled.println(steer, 1);
    if (mpuOk) { oled.print(F("Yaw:")); oled.println(yaw, 1); }
    oled.println(commandStale ? F("LINK: STALE") : F("LINK: OK"));
    oled.display();
  }
}
