/* * ESP32 4WD Robot - Software PWM Control (Previous Pins)
 * Optimized for JGB37-520 Motors
 */

// -------------------- PIN DEFINITIONS --------------------
// Front Left
const int FL_PWM = 13; const int FL_IN1 = 12; const int FL_IN2 = 14;
const int FL_ENC_A = 34; const int FL_ENC_B = 35;

// Front Right
const int FR_PWM = 27; const int FR_IN1 = 26; const int FR_IN2 = 25;
const int FR_ENC_A = 36; const int FR_ENC_B = 39;

// Rear Left
const int RL_PWM = 21;  const int RL_IN1 = 32; const int RL_IN2 = 15;
const int RL_ENC_A = 18; const int RL_ENC_B = 19;

// Rear Right
const int RR_PWM = 22;  const int RR_IN1 = 16; const int RR_IN2 = 17;
const int RR_ENC_A = 23; const int RR_ENC_B = 5;

// -------------------- GLOBALS --------------------
volatile long counts[4] = {0, 0, 0, 0}; 
int speeds[4] = {0, 0, 0, 0};           
const unsigned long PWM_PERIOD_US = 10000; 

// Motor inversion flags
bool invertMotor[4] = {false, false, false, false}; 

// -------------------- ENCODER ISRs --------------------
void IRAM_ATTR isrFL() { if(digitalRead(FL_ENC_A) == digitalRead(FL_ENC_B)) counts[0]--; else counts[0]++; }
void IRAM_ATTR isrFR() { if(digitalRead(FR_ENC_A) == digitalRead(FR_ENC_B)) counts[1]--; else counts[1]++; }
void IRAM_ATTR isrRL() { if(digitalRead(RL_ENC_A) == digitalRead(RL_ENC_B)) counts[2]--; else counts[2]++; }
void IRAM_ATTR isrRR() { if(digitalRead(RR_ENC_A) == digitalRead(RR_ENC_B)) counts[3]--; else counts[3]++; }

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);

  // Motor control pins
  int outPins[] = {FL_PWM, FL_IN1, FL_IN2, FR_PWM, FR_IN1, FR_IN2,
                   RL_PWM, RL_IN1, RL_IN2, RR_PWM, RR_IN1, RR_IN2};
  for(int p : outPins) pinMode(p, OUTPUT);

  // Encoder pins (34-39 strictly need external pullups if not on board)
  pinMode(FL_ENC_A, INPUT); pinMode(FL_ENC_B, INPUT);
  pinMode(FR_ENC_A, INPUT); pinMode(FR_ENC_B, INPUT);
  pinMode(RL_ENC_A, INPUT_PULLUP); pinMode(RL_ENC_B, INPUT_PULLUP);
  pinMode(RR_ENC_A, INPUT_PULLUP); pinMode(RR_ENC_B, INPUT_PULLUP);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(FL_ENC_A), isrFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_ENC_A), isrFR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RL_ENC_A), isrRL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_ENC_A), isrRR, CHANGE);

  Serial.println("System Initialized (Old Pins). Starting tests...");
}

// -------------------- MOTOR LOGIC --------------------

void software_pwm_loop() {
  unsigned long now = micros();
  unsigned long us_in_period = now % PWM_PERIOD_US;

  int pwmPins[] = {FL_PWM, FR_PWM, RL_PWM, RR_PWM};

  for(int i = 0; i < 4; i++) {
    long threshold = map(speeds[i], 0, 255, 0, PWM_PERIOD_US);
    digitalWrite(pwmPins[i], (us_in_period < threshold) ? HIGH : LOW);
  }
}

void setDirection(bool forward) {
  int in1[4] = {FL_IN1, FR_IN1, RL_IN1, RR_IN1};
  int in2[4] = {FL_IN2, FR_IN2, RL_IN2, RR_IN2};

  for(int i = 0; i < 4; i++){
    bool fwd = forward ^ invertMotor[i]; 
    digitalWrite(in1[i], fwd ? LOW : HIGH);
    digitalWrite(in2[i], fwd ? HIGH : LOW);
  }
}

void drive(int power) {
  for(int i = 0; i < 4; i++) speeds[i] = power;
}

void stopMotors() {
  drive(0);
  digitalWrite(FL_PWM, LOW); digitalWrite(FR_PWM, LOW);
  digitalWrite(RL_PWM, LOW); digitalWrite(RR_PWM, LOW);
}

// -------------------- MAIN LOOP --------------------
void loop() {
  int testPower = 110; // PWM value (0-255)

  // 1. Forward Test
  for(int i = 0; i < 4; i++) counts[i] = 0;
  Serial.println("\n[ACTION] MOVING FORWARD");
  setDirection(true);
  drive(testPower);

  unsigned long timer = millis();
  while(millis() - timer < 2000) software_pwm_loop();

  stopMotors();
  printReport();
  delay(2000);

  // 2. Reverse Test
  for(int i = 0; i < 4; i++) counts[i] = 0;
  Serial.println("\n[ACTION] MOVING REVERSE");
  setDirection(false);
  drive(testPower);

  timer = millis();
  while(millis() - timer < 2000) software_pwm_loop();

  stopMotors();
  printReport();
  delay(5000); 
}

void printReport() {
  Serial.println("--- ENCODER REPORT ---");
  Serial.printf("FRONT: L=%ld | R=%ld\n", counts[0], counts[1]);
  Serial.printf("REAR : L=%ld | R=%ld\n", counts[2], counts[3]);
  Serial.println("----------------------");
}