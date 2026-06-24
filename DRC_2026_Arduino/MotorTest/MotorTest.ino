// Kiwi Drive Motor Test
// M1 = Front-left, M2 = Front-right, M3 = Rear

// --- Pin definitions ---
const int M1_EN  = 9;   // PWM
const int M1_IN1 = 2;
const int M1_IN2 = 4;

const int M2_EN  = 10;  // PWM
const int M2_IN1 = 8;
const int M2_IN2 = 5;   // Was Pin 7

const int M3_EN  = 6;   // PWM
const int M3_IN1 = 13;
const int M3_IN2 = 12;

// --- Helper functions ---

void setMotor(int en, int in1, int in2, int speed) {
  // speed: -255 to 255, 0 = stop
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

// --- Setup ---

void setup() {
  pinMode(M1_EN,  OUTPUT); pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_EN,  OUTPUT); pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  pinMode(M3_EN,  OUTPUT); pinMode(M3_IN1, OUTPUT); pinMode(M3_IN2, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("Kiwi drive motor test starting...");
  stopAll();
  delay(2000); // pause before starting
}

// --- Main test loop ---

void loop() {

  // Test M1
  Serial.println("M1 forward (half speed)");
  setMotor(M1_EN, M1_IN1, M1_IN2, 128);
  delay(2000);
  Serial.println("M1 reverse (half speed)");
  setMotor(M1_EN, M1_IN1, M1_IN2, -128);
  delay(2000);
  stopAll(); delay(500);

  // Test M2
  Serial.println("M2 forward (half speed)");
  setMotor(M2_EN, M2_IN1, M2_IN2, 128);
  delay(2000);
  Serial.println("M2 reverse (half speed)");
  setMotor(M2_EN, M2_IN1, M2_IN2, -128);
  delay(2000);
  stopAll(); delay(500);

  // Test M3
  Serial.println("M3 forward (half speed)");
  setMotor(M3_EN, M3_IN1, M3_IN2, 128);
  delay(2000);
  Serial.println("M3 reverse (half speed)");
  setMotor(M3_EN, M3_IN1, M3_IN2, -128);
  delay(2000);
  stopAll(); delay(500);

  // Test all together at full speed
  Serial.println("All motors forward (full speed)");
  setMotor(M1_EN, M1_IN1, M1_IN2, 255);
  setMotor(M2_EN, M2_IN1, M2_IN2, 255);
  setMotor(M3_EN, M3_IN1, M3_IN2, 255);
  delay(3000);
  stopAll();

  // Test all together at full speed
  Serial.println("All motors forward (full speed)");
  setMotor(M1_EN, M1_IN1, M1_IN2, -255);
  setMotor(M2_EN, M2_IN1, M2_IN2, -255);
  setMotor(M3_EN, M3_IN1, M3_IN2, -255);
  delay(3000);
  stopAll();

  Serial.println("Test complete. Waiting 5 seconds...");
  delay(5000);
}