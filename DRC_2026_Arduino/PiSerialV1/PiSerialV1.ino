#include servo

// Kiwi Drive - Simplified (drive + turn)
// M1 = Front-left, M2 = Front-right, M3 = Rear
// RPi sends commands over USB serial in format: "D0.75T-0.5\n"
// D = drive (-1.0 to 1.0), T = turn (-1.0 to 1.0)

// --- Pin definitions ---


// --- Timeout ---
const unsigned long TIMEOUT_MS = 500; // stop if no command for 500ms
unsigned long lastCommandTime = 0;

// --- Serial parsing ---
String inputBuffer = "";

// --- Motor helper ---
void setMotor(int en, int in1, int in2, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(en, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(en, -speed);
  } else {
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

// --- Apply drive + turn values ---
void applyCommand(float drive, float turn) {
  // M1 and M2 handle forward/backward
  int drivePower = (int)(drive * 255);

  // M3 handles turning — positive turn = turn right (rear pushes left)
  int turnPower = (int)(turn * 255);

  setMotor(M1_EN, M1_IN1, M1_IN2, drivePower);
  setMotor(M2_EN, M2_IN1, M2_IN2, drivePower);
  setMotor(M3_EN, M3_IN1, M3_IN2, turnPower);
}

// --- Parse incoming serial command ---
// Expected format: "D0.75T-0.5\n"
void parseCommand(String cmd) {
  cmd.trim();
  int dIndex = cmd.indexOf('D');
  int tIndex = cmd.indexOf('T');

  if (dIndex == -1 || tIndex == -1) {
    Serial.println("ERR: bad format");
    return;
  }

  float drive = cmd.substring(dIndex + 1, tIndex).toFloat();
  float turn  = cmd.substring(tIndex + 1).toFloat();

  drive = constrain(drive, -1.0, 1.0);
  turn  = constrain(turn,  -1.0, 1.0);

  applyCommand(drive, turn);
  lastCommandTime = millis();

  Serial.print("OK D="); Serial.print(drive);
  Serial.print(" T="); Serial.println(turn);
}

// --- Setup ---
void setup() {
  pinMode(M1_EN, OUTPUT); pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_EN, OUTPUT); pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  pinMode(M3_EN, OUTPUT); pinMode(M3_IN1, OUTPUT); pinMode(M3_IN2, OUTPUT);

  Serial.begin(9600);
  Serial.println("Ready");
  stopAll();
  lastCommandTime = millis();
}

// --- Loop ---
void loop() {
  // Read serial
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      parseCommand(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  // Timeout check
  if (millis() - lastCommandTime > TIMEOUT_MS) {
    stopAll();
  }
}