/*
 * Elderly Bot ESP32 Firmware - Original Working Version
 * Uses standard rosserial WiFiClient + WiFiManager
 * No custom RosWiFiHardware class — avoids the operator= error
 * Compatible with rosserial_python serial_node.py tcp
 */

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

// ==================== PIN DEFINITIONS (exact from your hardware) ====================
const int FL_PWM = 13; const int FL_IN1 = 12; const int FL_IN2 = 14;
const int FL_ENC_A = 34; const int FL_ENC_B = 35;

const int FR_PWM = 27; const int FR_IN1 = 26; const int FR_IN2 = 25;
const int FR_ENC_A = 36; const int FR_ENC_B = 39;

const int RL_PWM = 21; const int RL_IN1 = 32; const int RL_IN2 = 15;
const int RL_ENC_A = 18; const int RL_ENC_B = 19;

const int RR_PWM = 22; const int RR_IN1 = 16; const int RR_IN2 = 17;
const int RR_ENC_A = 23; const int RR_ENC_B = 5;

// ==================== ROBOT KINEMATICS ====================
const float TRACK_WIDTH = 0.26;      // meters
const float WHEEL_RADIUS = 0.0325;   // meters
const float TICKS_PER_REV = 3960.0;
const float DISTANCE_PER_TICK = (2 * M_PI * WHEEL_RADIUS) / TICKS_PER_REV;

// ==================== PWM & SPEED ====================
const int PWM_FREQ = 5000;
const int PWM_RES = 8;
const int PWM_MAX = 255;
const float MAX_SPEED = 0.25;  // m/s

// ==================== ENCODER VARIABLES ====================
volatile long counts[4] = {0, 0, 0, 0}; // FL, FR, RL, RR
unsigned long last_interrupt_time[4] = {0, 0, 0, 0};

// ==================== ODOMETRY ====================
float odom_x = 0.0;
float odom_y = 0.0;
float odom_theta = 0.0;

float target_linear_x = 0.0;
float target_angular_z = 0.0;

// ==================== ROS ====================
ros::NodeHandle nh;
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);

void cmd_vel_cb(const geometry_msgs::Twist& twist_msg) {
  target_linear_x = twist_msg.linear.x;
  target_angular_z = twist_msg.angular.z;
}
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmd_vel_cb);

unsigned long last_odom_time = 0;
const int ODOM_PUBLISH_INTERVAL = 50; // 20 Hz

// ==================== INTERRUPT SERVICE ROUTINES ====================
void IRAM_ATTR isrFL() {
  unsigned long now = micros();
  if (now - last_interrupt_time[0] > 100) {
    int A = digitalRead(FL_ENC_A);
    int B = digitalRead(FL_ENC_B);
    if (A == B) counts[0]--; else counts[0]++;
    last_interrupt_time[0] = now;
  }
}

void IRAM_ATTR isrFR() {
  unsigned long now = micros();
  if (now - last_interrupt_time[1] > 100) {
    int A = digitalRead(FR_ENC_A);
    int B = digitalRead(FR_ENC_B);
    if (A == B) counts[1]--; else counts[1]++;
    last_interrupt_time[1] = now;
  }
}

void IRAM_ATTR isrRL() {
  unsigned long now = micros();
  if (now - last_interrupt_time[2] > 100) {
    int A = digitalRead(RL_ENC_A);
    int B = digitalRead(RL_ENC_B);
    if (A == B) counts[2]--; else counts[2]++;
    last_interrupt_time[2] = now;
  }
}

void IRAM_ATTR isrRR() {
  unsigned long now = micros();
  if (now - last_interrupt_time[3] > 100) {
    int A = digitalRead(RR_ENC_A);
    int B = digitalRead(RR_ENC_B);
    if (A == B) counts[3]--; else counts[3]++;
    last_interrupt_time[3] = now;
  }
}

// ==================== UPDATE ODOMETRY ====================
void updateOdometry(float dt) {
  long delta_left = (counts[0] + counts[2]) / 2;
  long delta_right = (counts[1] + counts[3]) / 2;

  // Reset counts
  for (int i = 0; i < 4; i++) counts[i] = 0;

  float delta_left_dist = delta_left * DISTANCE_PER_TICK;
  float delta_right_dist = delta_right * DISTANCE_PER_TICK;

  float delta_distance = (delta_left_dist + delta_right_dist) / 2.0;
  float delta_theta = (delta_right_dist - delta_left_dist) / TRACK_WIDTH;

  float dx = delta_distance * cos(odom_theta + delta_theta / 2.0);
  float dy = delta_distance * sin(odom_theta + delta_theta / 2.0);

  odom_x += dx;
  odom_y += dy;
  odom_theta += delta_theta;

  // Normalize angle
  while (odom_theta > M_PI) odom_theta -= 2 * M_PI;
  while (odom_theta < -M_PI) odom_theta += 2 * M_PI;

  // Publish velocities
  odom_msg.twist.twist.linear.x = delta_distance / dt;
  odom_msg.twist.twist.angular.z = delta_theta / dt;
}

// ==================== PUBLISH ODOMETRY ====================
void publishOdometry() {
  odom_msg.header.stamp = nh.now();
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_footprint";

  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;
  odom_msg.pose.pose.position.z = 0.0;

  // Manual yaw → quaternion conversion (rosserial tf doesn't have createQuaternionFromYaw)
  float half_yaw = odom_theta * 0.5;
  odom_msg.pose.pose.orientation.w = cos(half_yaw);
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sin(half_yaw);

  // Simple covariance (tune later if needed)
  for (int i = 0; i < 36; i++) {
    odom_msg.pose.covariance[i] = 0.0;
    odom_msg.twist.covariance[i] = 0.0;
  }
  odom_msg.pose.covariance[0]  = 0.01;   // x variance
  odom_msg.pose.covariance[7]  = 0.01;   // y variance
  odom_msg.pose.covariance[35] = 0.1;    // yaw variance (higher due to slip)

  odom_pub.publish(&odom_msg);
}

// ==================== MOTOR CONTROL TASK ====================
TaskHandle_t motorTaskHandle;

void motorControlTask(void * parameter) {
  for (;;) {
    float v_left = target_linear_x - target_angular_z * (TRACK_WIDTH / 2.0);
    float v_right = target_linear_x + target_angular_z * (TRACK_WIDTH / 2.0);

    int pwm_left = constrain((int)(v_left * PWM_MAX / MAX_SPEED), -PWM_MAX, PWM_MAX);
    int pwm_right = constrain((int)(v_right * PWM_MAX / MAX_SPEED), -PWM_MAX, PWM_MAX);

    // Left motors
    if (pwm_left > 0) {
      digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
      digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, LOW);
      ledcWrite(0, pwm_left); ledcWrite(2, pwm_left);
    } else if (pwm_left < 0) {
      digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
      digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, HIGH);
      ledcWrite(0, -pwm_left); ledcWrite(2, -pwm_left);
    } else {
      ledcWrite(0, 0); ledcWrite(2, 0);
    }

    // Right motors
    if (pwm_right > 0) {
      digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
      digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
      ledcWrite(1, pwm_right); ledcWrite(3, pwm_right);
    } else if (pwm_right < 0) {
      digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
      digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH);
      ledcWrite(1, -pwm_right); ledcWrite(3, -pwm_right);
    } else {
      ledcWrite(1, 0); ledcWrite(3, 0);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); // 100 Hz
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(FL_PWM, OUTPUT); pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
  pinMode(FR_PWM, OUTPUT); pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT);
  pinMode(RL_PWM, OUTPUT); pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT);
  pinMode(RR_PWM, OUTPUT); pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT);

  // PWM setup
  ledcSetup(0, PWM_FREQ, PWM_RES); ledcAttachPin(FL_PWM, 0);
  ledcSetup(1, PWM_FREQ, PWM_RES); ledcAttachPin(FR_PWM, 1);
  ledcSetup(2, PWM_FREQ, PWM_RES); ledcAttachPin(RL_PWM, 2);
  ledcSetup(3, PWM_FREQ, PWM_RES); ledcAttachPin(RR_PWM, 3);

  // Encoders
  pinMode(FL_ENC_A, INPUT); pinMode(FL_ENC_B, INPUT);
  pinMode(FR_ENC_A, INPUT); pinMode(FR_ENC_B, INPUT);
  pinMode(RL_ENC_A, INPUT_PULLUP); pinMode(RL_ENC_B, INPUT_PULLUP);
  pinMode(RR_ENC_A, INPUT_PULLUP); pinMode(RR_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(FL_ENC_A), isrFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_ENC_A), isrFR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RL_ENC_A), isrRL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_ENC_A), isrRR, CHANGE);

  // WiFiManager
  WiFiManager wifiManager;
  wifiManager.autoConnect("ElderlyBotESP32");

  // ROS setup
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(odom_pub);

  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_footprint";

  // Motor control task on Core 1
  xTaskCreatePinnedToCore(
    motorControlTask,
    "MotorTask",
    2048,
    NULL,
    1,
    &motorTaskHandle,
    1
  );
}

// ==================== LOOP ====================
void loop() {
  unsigned long now = millis();

  nh.spinOnce();

  // Publish odometry every 50ms (20 Hz)
  if (now - last_odom_time >= ODOM_PUBLISH_INTERVAL) {
    float dt = (now - last_odom_time) / 1000.0;
    updateOdometry(dt);
    publishOdometry();
    last_odom_time = now;
  }

  delay(1); // prevent watchdog
}