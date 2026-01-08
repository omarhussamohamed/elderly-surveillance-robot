/* * ESP32 4WD Robot - High Power Synchronization
 * Goal: Equal PWM to all motors, Correct Directions, Accurate Encoders
 */

// -------------------- PIN DEFINITIONS --------------------
const int FL_PWM = 13; const int FL_IN1 = 12; const int FL_IN2 = 14;
const int FL_ENC_A = 34; const int FL_ENC_B = 35;

const int FR_PWM = 27; const int FR_IN1 = 26; const int FR_IN2 = 25;
const int FR_ENC_A = 36; const int FR_ENC_B = 39;

const int RL_PWM = 21; const int RL_IN1 = 32; const int RL_IN2 = 15;
const int RL_ENC_A = 18; const int RL_ENC_B = 19;

const int RR_PWM = 22; const int RR_IN1 = 16; const int RR_IN2 = 17;
const int RR_ENC_A = 23; const int RR_ENC_B = 5;

// -------------------- SETTINGS --------------------
const int MIN_MOVE_PWM = 85;  // Higher floor to prevent stalling/beeping
const int PWM_FREQ = 5000;
const int PWM_RES = 8;
const int CH_FL = 0; const int CH_FR = 1; const int CH_RL = 4; const int CH_RR = 5;

volatile long counts[4] = {0, 0, 0, 0}; 

// -------------------- ENCODER ISRs (Optimized) --------------------
// These ensure Forward = Positive across all 4 wheels
void IRAM_ATTR isrFL() { digitalRead(FL_ENC_A) == digitalRead(FL_ENC_B) ? counts[0]-- : counts[0]++; }
void IRAM_ATTR isrFR() { digitalRead(FR_ENC_A) == digitalRead(FR_ENC_B) ? counts[1]-- : counts[1]++; }
void IRAM_ATTR isrRL() { digitalRead(RL_ENC_A) == digitalRead(RL_ENC_B) ? counts[2]-- : counts[2]++; }
void IRAM_ATTR isrRR() { digitalRead(RR_ENC_A) == digitalRead(RR_ENC_B) ? counts[3]-- : counts[3]++; }

// -------------------- MOTOR CONTROL --------------------

void setMotorDirection(bool forward) {
  if (forward) {
    // Forward Pins Configuration
    digitalWrite(FL_IN1, LOW);  digitalWrite(FL_IN2, HIGH);
    digitalWrite(FR_IN1, LOW);  digitalWrite(FR_IN2, HIGH);
    digitalWrite(RL_IN1, LOW);  digitalWrite(RL_IN2, HIGH);
    digitalWrite(RR_IN1, LOW);  digitalWrite(RR_IN2, HIGH);
  } else {
    // Reverse Pins Configuration
    digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
    digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
    digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, LOW);
    digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
  }
}

void drive(int pwr) {
  if (pwr == 0) {
    ledcWrite(CH_FL, 0); ledcWrite(CH_FR, 0);
    ledcWrite(CH_RL, 0); ledcWrite(CH_RR, 0);
    return;
  }
  // Ensure power is above the 'stall' threshold
  int effective_pwr = map(pwr, 1, 255, MIN_MOVE_PWM, 255);
  
  ledcWrite(CH_FL, effective_pwr);
  ledcWrite(CH_FR, effective_pwr);
  ledcWrite(CH_RL, effective_pwr);
  ledcWrite(CH_RR, effective_pwr);
}

void stopMotors() {
  drive(0);
  int inPins[] = {FL_IN1, FL_IN2, FR_IN1, FR_IN2, RL_IN1, RL_IN2, RR_IN1, RR_IN2};
  for(int p : inPins) digitalWrite(p, LOW);
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);
  
  // Safe Boot Delay
  pinMode(FL_IN1, INPUT); pinMode(RL_IN2, INPUT); delay(500);

  // Initialize all Outputs
  int pins[] = {FL_IN1, FL_IN2, FR_IN1, FR_IN2, RL_IN1, RL_IN2, RR_IN1, RR_IN2, FL_PWM, FR_PWM, RL_PWM, RR_PWM};
  for(int p : pins) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }

  // HW PWM Setup
  ledcSetup(CH_FL, PWM_FREQ, PWM_RES); ledcSetup(CH_FR, PWM_FREQ, PWM_RES);
  ledcSetup(CH_RL, PWM_FREQ, PWM_RES); ledcSetup(CH_RR, PWM_FREQ, PWM_RES);
  ledcAttachPin(FL_PWM, CH_FL); ledcAttachPin(FR_PWM, CH_FR);
  ledcAttachPin(RL_PWM, CH_RL); ledcAttachPin(RR_PWM, CH_RR);

  // Encoders (Internal Pullups for RL/RR, External 1k for FR)
  pinMode(FL_ENC_A, INPUT); pinMode(FL_ENC_B, INPUT);
  pinMode(FR_ENC_A, INPUT); pinMode(FR_ENC_B, INPUT);
  pinMode(RL_ENC_A, INPUT_PULLUP); pinMode(RL_ENC_B, INPUT_PULLUP);
  pinMode(RR_ENC_A, INPUT_PULLUP); pinMode(RR_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(FL_ENC_A), isrFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_ENC_A), isrFR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RL_ENC_A), isrRL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_ENC_A), isrRR, CHANGE);

  Serial.println("--- 4WD SYNC TEST READY ---");
}

// -------------------- LOOP --------------------
void loop() {
  // Test Forward
  Serial.println("\n[ACTION] FORWARD - EXPECT POSITIVE COUNTS");
  for(int i=0; i<4; i++) counts[i] = 0;
  setMotorDirection(true);
  drive(180); // Strong power for testing
  
  unsigned long start = millis();
  while(millis() - start < 3000) {
    Serial.printf("FL:%ld | FR:%ld | RL:%ld | RR:%ld\n", counts[0], counts[1], counts[2], counts[3]);
    delay(250);
  }
  stopMotors();
  delay(2000);

  // Test Reverse
  Serial.println("\n[ACTION] REVERSE - EXPECT NEGATIVE COUNTS");
  for(int i=0; i<4; i++) counts[i] = 0;
  setMotorDirection(false);
  drive(180);
  
  start = millis();
  while(millis() - start < 3000) {
    Serial.printf("FL:%ld | FR:%ld | RL:%ld | RR:%ld\n", counts[0], counts[1], counts[2], counts[3]);
    delay(250);
  }
  stopMotors();
  delay(5000);
}