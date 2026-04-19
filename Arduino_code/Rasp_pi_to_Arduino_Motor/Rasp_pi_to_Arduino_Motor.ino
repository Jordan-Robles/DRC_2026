const int pwmPin = 2;  // RC signal pin
const int RPWM = 6;   // PWM pin for forward
const int LPWM = 5;   // PWM pin for reverse
const int REN  = 7;   // Enable forward
const int LEN  = 8;   // Enable reverse
unsigned long pwmValue = 0;
int mappedValue = 0;

void setup() {
  Serial.begin(9600);
  pinMode(pwmPin, INPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);

    stop_motor();
}

// // Converts 0-100% to 0-255 PWM
int percentToPWM(int percent) {
  percent = constrain(percent, 0, 100);
  return map(percent, 0, 100, 0, 255);
}

void stop_motor() {
  digitalWrite(REN, LOW);
  digitalWrite(LEN, LOW);
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}

void move_forward(int speed_percent = 100) {
  stop_motor();
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);
  analogWrite(RPWM, percentToPWM(speed_percent));
  analogWrite(LPWM, 0);
}

int mapForward(int input) {
  input = constrain(input, 0, 50); // Clamp input to valid range
  return map(input, 50, 0, 0, 100); // Reverse map: 50->0, 0->100
}

void move_reverse(int speed_percent = 100) {
  stop_motor();
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);
  analogWrite(LPWM, percentToPWM(speed_percent));
  analogWrite(RPWM, 0);
}

int mapReverse(int input) {
  input = constrain(input, 50, 100);
  return map(input, 50, 100, 0, 100);
}

void loop() {
  if (Serial.available()) {
    int input = Serial.parseInt(); // Expecting an int 0–100
    input = constrain(input, 0, 100);
    mappedValue = input;
    Serial.print(mappedValue);

    if (mappedValue < 45) {
      move_forward(mapForward(mappedValue));
    } else if (mappedValue > 55) {
      move_reverse(mapReverse(mappedValue));
    } else {
      stop_motor();
    }
  }
  // delay(100); // Delay between readings
}




