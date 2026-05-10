// RC Transmitter driven version
// Steering PWM on pin 3, Throttle PWM on pin 7

// --- Pin definitions for the L298N ---
const int M1_EN  = 9;
const int M1_IN1 = 2;
const int M1_IN2 = 4;

const int M2_EN  = 10;
const int M2_IN1 = 5;
const int M2_IN2 = 8;

const int M3_EN  = 6;
const int M3_IN1 = 12;
const int M3_IN2 = 13;

// --- RC Receiver input pins ---
const int RC_STEERING_PIN = 3;
const int RC_THROTTLE_PIN = 7;

// --- Config ---
const int TURN_SCALE  = 255;   // max turning power
const int DRIVE_SPEED = 255;   // max drive speed (0–255)

// --- RC PWM calibration (microseconds) ---
// Adjust these to match your transmitter's actual output
const int RC_MIN      = 1000;  // full reverse / full left
const int RC_MID      = 1500;  // centre / neutral
const int RC_MAX      = 2000;  // full forward / full right
const int RC_DEADBAND = 50;    // µs either side of centre to ignore

// --- Motor function ---
void setMotor(int en, int in1, int in2, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(en, speed);
  }
  else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(en, -speed);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(en, 0);
  }
}

void stopAll() {
  setMotor(M1_EN, M1_IN1, M1_IN2, 0);
  setMotor(M2_EN, M2_IN1, M2_IN2, 0);
  setMotor(M3_EN, M3_IN1, M3_IN2, 0);
}

// --- Read RC PWM pulse width (microseconds) ---
// Returns -1 if signal is absent or invalid
long readPWM(int pin) {
  unsigned long pulse = pulseIn(pin, HIGH, 25000); // 25ms timeout
  if (pulse == 0) return -1;
  return (long)pulse;
}

// --- Map PWM to a -1.0 to 1.0 float with deadband ---
float pwmToFloat(long pwm) {
  if (pwm < 0) return 0.0;  // no signal

  pwm = constrain(pwm, RC_MIN, RC_MAX);

  // Apply deadband around centre
  if (abs(pwm - RC_MID) < RC_DEADBAND) return 0.0;

  // Map directly: 1000 -> -1.0, 1500 -> 0.0, 2000 -> 1.0
  return (float)(pwm - RC_MID) / (float)(RC_MAX - RC_MID);
}

// --- Setup ---
void setup() {
  pinMode(M1_EN, OUTPUT); pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_EN, OUTPUT); pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  pinMode(M3_EN, OUTPUT); pinMode(M3_IN1, OUTPUT); pinMode(M3_IN2, OUTPUT);

  pinMode(RC_STEERING_PIN, INPUT);
  pinMode(RC_THROTTLE_PIN, INPUT);

  Serial.begin(9600);
  Serial.println("RC Ready");
}

// --- Loop ---
void loop() {

  // --- Read RC channels ---
  long steeringPWM = readPWM(RC_STEERING_PIN);
  long throttlePWM = readPWM(RC_THROTTLE_PIN);

  float steeringVal = pwmToFloat(steeringPWM);   // -1.0 (left) to 1.0 (right)
  float throttleVal = pwmToFloat(throttlePWM);   // -1.0 (reverse) to 1.0 (forward)

  // --- Failsafe: no signal detected ---
  if (steeringPWM < 0 || throttlePWM < 0) {
    stopAll();
    return;
  }

// --- Drive motors M1 + M2 from throttle ---
  int driveSpeed = (int)(throttleVal * DRIVE_SPEED);
  setMotor(M1_EN, M1_IN1, M1_IN2, driveSpeed);
  setMotor(M2_EN, M2_IN1, M2_IN2, driveSpeed);

  // --- Steering motor M3 ---
  int turnPower = (int)(-(steeringVal * TURN_SCALE));
  if (turnPower != 0 && abs(turnPower) < 150) {
    turnPower = (turnPower > 0) ? 150 : -150;
  }
  setMotor(M3_EN, M3_IN1, M3_IN2, turnPower);

  // --- Send steering to Pi ---
  Serial.println(steeringVal, 4);
}