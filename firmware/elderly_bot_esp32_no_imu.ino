/*
 * ESP32 Firmware WITHOUT IMU Support
 * 
 * Use this version if MPU9250 is connected directly to Jetson I2C.
 * This simplifies the ESP32 firmware to only handle motor control and odometry.
 * 
 * To use:
 * 1. Rename this file to elderly_bot_esp32.ino
 * 2. Remove or comment out all IMU-related code
 * 3. Flash to ESP32
 * 
 * Benefits:
 * - Smaller firmware size
 * - Less RAM usage
 * - Simpler code (easier to debug)
 * - Reduced serial communication load
 * - IMU runs independently on Jetson
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ==================== SETTINGS ====================
#define ROS_SERIAL_BUFFER_SIZE 1024

// Rate control (Hz)
#define ODOM_PUBLISH_RATE 20       // 20Hz odometry
#define CONTROL_LOOP_RATE 50       // 50Hz control loop

// ==================== PINOUT (FINAL VERIFIED) ====================
#define MOTOR_FL_PWM 13
#define MOTOR_FL_IN1 12
#define MOTOR_FL_IN2 14
#define MOTOR_FR_PWM 27
#define MOTOR_FR_IN1 26
#define MOTOR_FR_IN2 25

#define MOTOR_RL_PWM 2  
#define MOTOR_RL_IN1 32
#define MOTOR_RL_IN2 15
#define MOTOR_RR_PWM 4  
#define MOTOR_RR_IN1 16
#define MOTOR_RR_IN2 17

// Encoders (Corrected Pins)
#define ENC_FL_A 34
#define ENC_FL_B 35
#define ENC_FR_A 36
#define ENC_FR_B 39
#define ENC_RL_A 18
#define ENC_RL_B 19
#define ENC_RR_A 23
#define ENC_RR_B 5

// ==================== GLOBALS ====================
ros::NodeHandle nh;

volatile long encoder_fl=0, encoder_fr=0, encoder_rl=0, encoder_rr=0;
long prev_encoder_fl=0, prev_encoder_fr=0, prev_encoder_rl=0, prev_encoder_rr=0;
int motor_speeds[4] = {0,0,0,0}; 

float target_vel_fl=0, target_vel_fr=0, target_vel_rl=0, target_vel_rr=0;
float current_vel_fl=0, current_vel_fr=0, current_vel_rl=0, current_vel_rr=0;
float error_sum_fl=0, error_sum_fr=0, error_sum_rl=0, error_sum_rr=0;
float prev_error_fl=0, prev_error_fr=0, prev_error_rl=0, prev_error_rr=0;

float odom_x = 0, odom_y = 0, odom_theta = 0;

// TWEAKED PID GAINS FOR STABILITY
float kp = 5.0, ki = 2.0, kd = 0.1; 
const float WHEEL_RADIUS = 0.0325;
const float TRACK_WIDTH = 0.26;
const int TICKS_PER_REV = 4900; 

unsigned long last_cmd_time = 0;

nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("wheel_odom", &odom_msg);

// Mutex for shared data
SemaphoreHandle_t encoder_mutex;
SemaphoreHandle_t motor_mutex;

// ==================== ISRs ====================
void IRAM_ATTR isrFL(){ 
  if(digitalRead(ENC_FL_A)==digitalRead(ENC_FL_B)) encoder_fl--; 
  else encoder_fl++; 
}
void IRAM_ATTR isrFR(){ 
  if(digitalRead(ENC_FR_A)==digitalRead(ENC_FR_B)) encoder_fr--; 
  else encoder_fr++; 
}
void IRAM_ATTR isrRL(){ 
  if(digitalRead(ENC_RL_A)==digitalRead(ENC_RL_B)) encoder_rl--; 
  else encoder_rl++; 
}
void IRAM_ATTR isrRR(){ 
  if(digitalRead(ENC_RR_A)==digitalRead(ENC_RR_B)) encoder_rr--; 
  else encoder_rr++; 
}

// ==================== MOTOR LOGIC ====================
// Software PWM for ALL motors (hardware PWM causes enable pin issues)
void software_pwm_loop() {
  unsigned long now = micros();
  unsigned long us_in_period = now % 1000; // 1kHz frequency
  
  // All motors use software PWM
  long fl_thresh = map(abs(motor_speeds[0]), 0, 255, 0, 1000);
  long fr_thresh = map(abs(motor_speeds[1]), 0, 255, 0, 1000);
  long rl_thresh = map(abs(motor_speeds[2]), 0, 255, 0, 1000);
  long rr_thresh = map(abs(motor_speeds[3]), 0, 255, 0, 1000);
  
  digitalWrite(MOTOR_FL_PWM, (us_in_period < fl_thresh) ? HIGH : LOW);
  digitalWrite(MOTOR_FR_PWM, (us_in_period < fr_thresh) ? HIGH : LOW);
  digitalWrite(MOTOR_RL_PWM, (us_in_period < rl_thresh) ? HIGH : LOW);
  digitalWrite(MOTOR_RR_PWM, (us_in_period < rr_thresh) ? HIGH : LOW);
}

void setMotorSpeed(int motor, int pwm_value) {
  if(xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    int in1, in2;
    if(motor == 0) { in1=MOTOR_FL_IN1; in2=MOTOR_FL_IN2; }
    else if(motor == 1) { in1=MOTOR_FR_IN1; in2=MOTOR_FR_IN2; }
    else if(motor == 2) { in1=MOTOR_RL_IN1; in2=MOTOR_RL_IN2; }
    else { in1=MOTOR_RR_IN1; in2=MOTOR_RR_IN2; }

    // Direction control for all motors
    if (pwm_value > 0) {
      digitalWrite(in1, (motor < 2 ? LOW : HIGH)); 
      digitalWrite(in2, (motor < 2 ? HIGH : LOW));
    } else if (pwm_value < 0) {
      digitalWrite(in1, (motor < 2 ? HIGH : LOW)); 
      digitalWrite(in2, (motor < 2 ? LOW : HIGH));
    } else {
      digitalWrite(in1, LOW); 
      digitalWrite(in2, LOW);
    }

    // Store PWM value for software PWM
    motor_speeds[motor] = pwm_value;
    xSemaphoreGive(motor_mutex);
  }
}

int computePID(float target, float current, float &error_sum, float &prev_error, float dt) {
  if (target == 0 && current == 0) return 0;
  float error = target - current;
  error_sum += error * dt;
  error_sum = constrain(error_sum, -50, 50);
  float d = (dt > 0) ? (error - prev_error) / dt : 0;
  prev_error = error;
  return constrain((int)(kp*error + ki*error_sum + kd*d), -255, 255);
}

void updateOdometry(float dt) {
  float v_left = (current_vel_fl + current_vel_rl) / 2.0;
  float v_right = (current_vel_fr + current_vel_rr) / 2.0;
  float v_linear = (v_left + v_right) / 2.0;
  float v_angular = (v_right - v_left) / TRACK_WIDTH;

  odom_theta += v_angular * dt;
  odom_x += v_linear * cos(odom_theta) * dt;
  odom_y += v_linear * sin(odom_theta) * dt;

  while(odom_theta > PI) odom_theta -= 2 * PI;
  while(odom_theta < -PI) odom_theta += 2 * PI;
}

void controlLoop() {
  const float dt = 1.0 / CONTROL_LOOP_RATE;
  const float ticks_to_m = (2.0 * PI * WHEEL_RADIUS) / TICKS_PER_REV;
  
  // Read encoder values atomically to avoid race conditions
  long enc_fl, enc_fr, enc_rl, enc_rr;
  if(xSemaphoreTake(encoder_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    enc_fl = encoder_fl; 
    enc_fr = encoder_fr;
    enc_rl = encoder_rl; 
    enc_rr = encoder_rr;
    xSemaphoreGive(encoder_mutex);
  } else {
    return; // Skip this iteration if mutex timeout
  }
  
  current_vel_fl = (enc_fl - prev_encoder_fl) * ticks_to_m / dt;
  current_vel_fr = (enc_fr - prev_encoder_fr) * ticks_to_m / dt;
  current_vel_rl = (enc_rl - prev_encoder_rl) * ticks_to_m / dt;
  current_vel_rr = (enc_rr - prev_encoder_rr) * ticks_to_m / dt;

  prev_encoder_fl = enc_fl; 
  prev_encoder_fr = enc_fr;
  prev_encoder_rl = enc_rl; 
  prev_encoder_rr = enc_rr;

  setMotorSpeed(0, computePID(target_vel_fl, current_vel_fl, error_sum_fl, prev_error_fl, dt));
  setMotorSpeed(1, computePID(target_vel_fr, current_vel_fr, error_sum_fr, prev_error_fr, dt));
  setMotorSpeed(2, computePID(target_vel_rl, current_vel_rl, error_sum_rl, prev_error_rl, dt));
  setMotorSpeed(3, computePID(target_vel_rr, current_vel_rr, error_sum_rr, prev_error_rr, dt));

  updateOdometry(dt);
}

void cmdVelCallback(const geometry_msgs::Twist& cmd_vel){
  last_cmd_time = millis();
  float v = cmd_vel.linear.x;
  float w = cmd_vel.angular.z;
  target_vel_fl = target_vel_rl = v - w * (TRACK_WIDTH / 2.0);
  target_vel_fr = target_vel_rr = v + w * (TRACK_WIDTH / 2.0);
}
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmdVelCallback);

// ==================== FreeRTOS TASKS ====================

// Task 1: Control Loop (50Hz)
void controlTask(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(1000 / CONTROL_LOOP_RATE);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  for(;;) {
    // Safety timeout: stop motors if no command received for 500ms
    unsigned long current_time = millis();
    if((current_time - last_cmd_time > 500) || (current_time < last_cmd_time)) {
      target_vel_fl = target_vel_fr = target_vel_rl = target_vel_rr = 0;
    }
    
    controlLoop();
    
    // Software PWM must run continuously
    software_pwm_loop();
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Task 2: Odometry Publisher (20Hz)
void odomTask(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(1000 / ODOM_PUBLISH_RATE);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  for(;;) {
    odom_msg.header.stamp = nh.now();
    odom_msg.pose.pose.position.x = odom_x;
    odom_msg.pose.pose.position.y = odom_y;
    odom_msg.pose.pose.orientation.z = sin(odom_theta * 0.5);
    odom_msg.pose.pose.orientation.w = cos(odom_theta * 0.5);
    
    // Calculate velocities for twist
    float v_left = (current_vel_fl + current_vel_rl) / 2.0;
    float v_right = (current_vel_fr + current_vel_rr) / 2.0;
    float v_linear = (v_left + v_right) / 2.0;
    float v_angular = (v_right - v_left) / TRACK_WIDTH;
    
    odom_msg.twist.twist.linear.x = v_linear;
    odom_msg.twist.twist.angular.z = v_angular;
    
    odom_pub.publish(&odom_msg);
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Task 3: ROS Communication (handles spinOnce)
void rosTask(void *pvParameters) {
  for(;;) {
    nh.spinOnce();
    vTaskDelay(pdMS_TO_TICKS(10)); // ~100Hz ROS communication
  }
}

void setup() {
  Serial.begin(115200);
  
  // Wait for Serial to be ready (non-blocking timeout)
  unsigned long start = millis();
  while(!Serial && (millis() - start < 2000)) {
    delay(10);
  }
  
  Serial.println("\n\nElderly Bot ESP32 Starting...");
  Serial.println("Firmware: FreeRTOS (NO IMU - IMU on Jetson)");
  
  // Create mutexes
  encoder_mutex = xSemaphoreCreateMutex();
  motor_mutex = xSemaphoreCreateMutex();
  
  if(encoder_mutex == NULL || motor_mutex == NULL) {
    Serial.println("ERROR: Failed to create mutexes!");
    while(1) delay(1000); // Halt
  }

  // All motor pins as OUTPUT (software PWM, no hardware PWM setup)
  int outPins[] = {MOTOR_FL_IN1, MOTOR_FL_IN2, MOTOR_FL_PWM, MOTOR_FR_IN1, MOTOR_FR_IN2, MOTOR_FR_PWM,
                   MOTOR_RL_IN1, MOTOR_RL_IN2, MOTOR_RL_PWM, MOTOR_RR_IN1, MOTOR_RR_IN2, MOTOR_RR_PWM};
  for(int p : outPins) pinMode(p, OUTPUT);
  
  // Initialize all PWM pins to LOW (software PWM)
  digitalWrite(MOTOR_FL_PWM, LOW);
  digitalWrite(MOTOR_FR_PWM, LOW);
  digitalWrite(MOTOR_RL_PWM, LOW);
  digitalWrite(MOTOR_RR_PWM, LOW);

  pinMode(ENC_FL_A, INPUT); pinMode(ENC_FL_B, INPUT);
  pinMode(ENC_FR_A, INPUT); pinMode(ENC_FR_B, INPUT);
  pinMode(ENC_RL_A, INPUT_PULLUP); pinMode(ENC_RL_B, INPUT_PULLUP);
  pinMode(ENC_RR_A, INPUT_PULLUP); pinMode(ENC_RR_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_FL_A), isrFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_A), isrFR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RL_A), isrRL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RR_A), isrRR, CHANGE);

  // Initialize ROS (NO IMU initialization)
  Serial.print("Initializing ROS...");
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  
  // Small delay to allow ROS sync packets
  delay(100);
  
  nh.subscribe(cmd_vel_sub);
  nh.advertise(odom_pub);
  
  // Initialize message frame_ids
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_footprint";
  
  Serial.println(" OK");
  Serial.println("Creating FreeRTOS tasks...");
  
  // Create FreeRTOS tasks (NO IMU task)
  xTaskCreatePinnedToCore(controlTask, "ControlTask", 4096, NULL, 3, NULL, 1);  // Core 1, priority 3
  xTaskCreatePinnedToCore(odomTask, "OdomTask", 4096, NULL, 2, NULL, 0);        // Core 0, priority 2
  xTaskCreatePinnedToCore(rosTask, "RosTask", 8192, NULL, 1, NULL, 0);          // Core 0, priority 1
  
  Serial.println("Tasks created!");
  Serial.println("Elderly Bot Ready!");
  Serial.println("Waiting for ROS connection...");
  Serial.println("NOTE: IMU must be connected to Jetson and mpu9250_node running!");
  
  // Allow tasks to start
  delay(100);
}

void loop() {
  // Empty - everything runs in FreeRTOS tasks
  vTaskDelay(pdMS_TO_TICKS(1000)); // Sleep to reduce CPU usage
}

