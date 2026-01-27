/*
 * Elderly Bot ESP32 Firmware - Original Working Version
 * Uses standard rosserial WiFiClient + WiFiManager
 * No custom RosWiFiHardware class
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

// ==================== PIN DEFINITIONS ====================
const int FL_PWM = 13; const int FL_IN1 = 12; const int FL_IN2 = 14;
const int FL_ENC_A = 34; const int FL_ENC_B = 35;

const int FR_PWM = 27; const int FR_IN1 = 26; const int FR_IN2 = 25;
const int FR_ENC_A = 36; const int FR_ENC_B = 39;

const int RL_PWM = 21; const int RL_IN1 = 32; const int RL_IN2 = 15;
const int RL_ENC_A = 18; const int RL_ENC_B = 19;

const int RR_PWM = 22; const int RR_IN1 = 16; const int RR_IN2 = 17;
const int RR_ENC_A = 23; const int RR_ENC_B = 5;

// ==================== ROBOT KINEMATICS ====================
const float TRACK_WIDTH = 0.26;
const float WHEEL_RADIUS = 0.0325;
const float TICKS_PER_REV = 3960.0;
const float DISTANCE_PER_TICK = (2 * M_PI * WHEEL_RADIUS) / TICKS_PER_REV;

// ==================== PWM & SPEED ====================
const int PWM_FREQ = 5000;
const int PWM_RES = 8;
const int PWM_MAX = 255;
const float MAX_SPEED = 0.25;

// ==================== ENCODERS ====================
volatile long counts[4] = {0, 0, 0, 0};
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

void cmd_vel_cb(const geometry_msgs::Twist& msg) {
  target_linear_x = msg.linear.x;
  target_angular_z = msg.angular.z;
}
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmd_vel_cb);

unsigned long last_odom_time = 0;
const int ODOM_PUBLISH_INTERVAL = 50;

// ==================== ISR ====================
void IRAM_ATTR isrFL() {
  unsigned long now = micros();
  if (now - last_interrupt_time[0] > 100) {
    if (digitalRead(FL_ENC_A) == digitalRead(FL_ENC_B)) counts[0]--;
    else counts[0]++;
    last_interrupt_time[0] = now;
  }
}

void IRAM_ATTR isrFR() {
  unsigned long now = micros();
  if (now - last_interrupt_time[1] > 100) {
    if (digitalRead(FR_ENC_A) == digitalRead(FR_ENC_B)) counts[1]--;
    else counts[1]++;
    last_interrupt_time[1] = now;
  }
}

void IRAM_ATTR isrRL() {
  unsigned long now = micros();
  if (now - last_interrupt_time[2] > 100) {
    if (digitalRead(RL_ENC_A) == digitalRead(RL_ENC_B)) counts[2]--;
    else counts[2]++;
    last_interrupt_time[2] = now;
  }
}

void IRAM_ATTR isrRR() {
  unsigned long now = micros();
  if (now - last_interrupt_time[3] > 100) {
    if (digitalRead(RR_ENC_A) == digitalRead(RR_ENC_B)) counts[3]--;
    else counts[3]++;
    last_interrupt_time[3] = now;
  }
}

// ==================== ODOM UPDATE ====================
void updateOdometry(float dt) {
  long dl = (counts[0] + counts[2]) / 2;
  long dr = (counts[1] + counts[3]) / 2;

  for (int i = 0; i < 4; i++) counts[i] = 0;

  float dl_d = dl * DISTANCE_PER_TICK;
  float dr_d = dr * DISTANCE_PER_TICK;

  float d = (dl_d + dr_d) / 2.0;
  float dth = (dr_d - dl_d) / TRACK_WIDTH;

  odom_x += d * cos(odom_theta + dth * 0.5);
  odom_y += d * sin(odom_theta + dth * 0.5);
  odom_theta += dth;

  while (odom_theta > M_PI) odom_theta -= 2 * M_PI;
  while (odom_theta < -M_PI) odom_theta += 2 * M_PI;

  odom_msg.twist.twist.linear.x = d / dt;
  odom_msg.twist.twist.angular.z = dth / dt;
}

// ==================== ODOM PUBLISH ====================
void publishOdometry() {
  odom_msg.header.stamp = nh.now();
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_footprint";

  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;

  float h = odom_theta * 0.5;
  odom_msg.pose.pose.orientation.w = cos(h);
  odom_msg.pose.pose.orientation.z = sin(h);

  odom_pub.publish(&odom_msg);
}

// ==================== MOTOR TASK ====================
TaskHandle_t motorTaskHandle;

void motorControlTask(void *p) {
  for (;;) {
    float vl = target_linear_x - target_angular_z * TRACK_WIDTH * 0.5;
    float vr = target_linear_x + target_angular_z * TRACK_WIDTH * 0.5;

    int pl = constrain(vl * PWM_MAX / MAX_SPEED, -PWM_MAX, PWM_MAX);
    int pr = constrain(vr * PWM_MAX / MAX_SPEED, -PWM_MAX, PWM_MAX);

    digitalWrite(FL_IN1, pl > 0); digitalWrite(FL_IN2, pl < 0);
    digitalWrite(RL_IN1, pl > 0); digitalWrite(RL_IN2, pl < 0);
    digitalWrite(FR_IN1, pr > 0); digitalWrite(FR_IN2, pr < 0);
    digitalWrite(RR_IN1, pr > 0); digitalWrite(RR_IN2, pr < 0);

    ledcWrite(0, abs(pl));
    ledcWrite(2, abs(pl));
    ledcWrite(1, abs(pr));
    ledcWrite(3, abs(pr));

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);

  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT);
  pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT);
  pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT);

  ledcSetup(0, PWM_FREQ, PWM_RES); ledcAttachPin(FL_PWM, 0);
  ledcSetup(1, PWM_FREQ, PWM_RES); ledcAttachPin(FR_PWM, 1);
  ledcSetup(2, PWM_FREQ, PWM_RES); ledcAttachPin(RL_PWM, 2);
  ledcSetup(3, PWM_FREQ, PWM_RES); ledcAttachPin(RR_PWM, 3);

  attachInterrupt(digitalPinToInterrupt(FL_ENC_A), isrFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_ENC_A), isrFR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RL_ENC_A), isrRL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_ENC_A), isrRR, CHANGE);

  // ==================== WiFiManager SMALL EDIT ====================
  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(0);      // keep portal alive
  wifiManager.setBreakAfterConfig(false);     // do not close after connect
  wifiManager.setConfigPortalBlocking(false); // firmware keeps running
  wifiManager.autoConnect("ElderlyBotESP32");

  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(odom_pub);

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

  if (now - last_odom_time >= ODOM_PUBLISH_INTERVAL) {
    float dt = (now - last_odom_time) / 1000.0;
    updateOdometry(dt);
    publishOdometry();
    last_odom_time = now;
  }

  delay(1);
}
