//THis code is for Serial communitcaion from Raspberry pi to the arduino, use this code for actual driving

// --- Pin definitions for the L298n ---
const int M1_EN  = 9;
const int M1_IN1 = 2;
const int M1_IN2 = 4;

const int M2_EN  = 10;
const int M2_IN1 = 5;
const int M2_IN2 = 8;

const int M3_EN  = 6;
const int M3_IN1 = 12;
const int M3_IN2 = 13;

// --- Config ---
const int DRIVE_SPEED = 200;   // forward speed for M1 + M2 (0–255) 170
const int TURN_SCALE  = 255;   // max turning power 255

// --- Serial ---
String inputBuffer = "";
float steeringAngle = 0;

// --- Motor funtion ---
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

// --- Setup ---
void setup() {
  pinMode(M1_EN, OUTPUT); pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_EN, OUTPUT); pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  pinMode(M3_EN, OUTPUT); pinMode(M3_IN1, OUTPUT); pinMode(M3_IN2, OUTPUT);

  Serial.begin(9600);
  Serial.println("Ready");
}

// --- Loop ---
void loop() {

  // --- Read serial angle from Pi ---
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      steeringAngle = inputBuffer.toFloat();
      inputBuffer = "";

      // clamp input, althought eh model already outputs a value between -1 and 1, we keep this here just in case
      steeringAngle = constrain(steeringAngle, -1.0, 1.0);

      Serial.print("Angle: ");
      Serial.println(steeringAngle);
    } 
    else {
      inputBuffer += c;
    }
  }

  // --- Drive forward (constant) ---
  setMotor(M1_EN, M1_IN1, M1_IN2, DRIVE_SPEED);
  setMotor(M2_EN, M2_IN1, M2_IN2, DRIVE_SPEED);

  // --- Steering (rear wheel) ---
int turnPower = (int)(steeringAngle * TURN_SCALE);
  if (turnPower != 0 && abs(turnPower) < 150) {
    turnPower = (turnPower > 0) ? 150 : -150;
  }

  setMotor(M3_EN, M3_IN1, M3_IN2, turnPower);
}