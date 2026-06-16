/*
  RC PWM → Serial  (Arduino Uno)
  ================================
  Reads a single PWM channel from your RC receiver (e.g. steering channel)
  and sends the pulse-width in microseconds as a plain integer line:
      1523\n

  Wire:
    RC receiver  CH1  →  Arduino Pin 2  (interrupt-capable)
    RC receiver  GND  →  Arduino GND
    (No shared power needed if receiver has its own supply)

  The Python Webots controller reads this over USB Serial.
*/

const int RC_PIN = 3;       // must be interrupt-capable on Uno (2 or 3)
const int BAUD   = 9600;

volatile unsigned long risingTime  = 0;
volatile unsigned int  pulseWidth  = 1500;   // default = centre
volatile bool          newPulse    = false;

// ── ISR ──────────────────────────────────────────────────────────────────────
void rcISR() {
  if (digitalRead(RC_PIN) == HIGH) {
    risingTime = micros();
  } else {
    unsigned long pw = micros() - risingTime;
    // Sanity-check: valid RC pulse is 800–2200 µs
    if (pw >= 800 && pw <= 2200) {
      pulseWidth = (unsigned int)pw;
      newPulse   = true;
    }
  }
}

// ── SETUP ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(BAUD);
  pinMode(RC_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RC_PIN), rcISR, CHANGE);
}

// ── LOOP ─────────────────────────────────────────────────────────────────────
void loop() {
  if (newPulse) {
    noInterrupts();
    unsigned int pw = pulseWidth;
    newPulse = false;
    interrupts();

    Serial.println(pw);   // e.g.  "1523\n"
  }
  // ~50 Hz max output – RC frames arrive every 20 ms anyway
  delay(18);
}
