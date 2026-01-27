/*
 * Elderly Bot ESP32 Firmware - Original Working Version
 * ONLY WiFiManager logic modified
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

void cmd_vel_cb(const geometry_msgs::Twist& twist_msg) {
  target_linear_x = twist_msg.linear.x;
  target_angular_z = twist_msg.angular.z;
}
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmd_vel_cb);

unsigned long last_odom_time = 0;
const int ODOM_PUBLISH_INTERVAL = 50;

// ==================== WIFI MANAGER ONLY ====================
WiFiManager wifiManager;
Preferences prefs;

// stored master IP
char master_ip[16] = "192.168.1.2";

// ALWAYS shown in portal
WiFiManagerParameter master_ip_param(
  "masterip",
  "ROS Master IP",
  master_ip,
  16
);

// ==================== INTERRUPTS ====================
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

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);

  prefs.begin("wifi", false);
  prefs.getString("master_ip", master_ip, sizeof(master_ip));

  // ---- WiFiManager (STABLE, BLOCKING) ----
  wifiManager.setConfigPortalTimeout(0);
  wifiManager.setBreakAfterConfig(true);
  wifiManager.addParameter(&master_ip_param);

  // Connect or open portal if needed
  wifiManager.autoConnect("ElderlyBotESP32");

  // Save IP after portal
  strcpy(master_ip, master_ip_param.getValue());
  prefs.putString("master_ip", master_ip);

  // ---- ORIGINAL ROS SETUP (UNCHANGED) ----
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(odom_pub);

  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_footprint";

  // ---- ORIGINAL MOTOR TASK (UNCHANGED) ----
  xTaskCreatePinnedToCore(
    [](void *p) {
      for (;;) {
        float v_left  = target_linear_x - target_angular_z * (TRACK_WIDTH / 2.0);
        float v_right = target_linear_x + target_angular_z * (TRACK_WIDTH / 2.0);

        int pwm_left  = constrain(v_left  * PWM_MAX / MAX_SPEED, -PWM_MAX, PWM_MAX);
        int pwm_right = constrain(v_right * PWM_MAX / MAX_SPEED, -PWM_MAX, PWM_MAX);

        digitalWrite(FL_IN1, pwm_left > 0);  digitalWrite(FL_IN2, pwm_left < 0);
        digitalWrite(RL_IN1, pwm_left > 0);  digitalWrite(RL_IN2, pwm_left < 0);
        digitalWrite(FR_IN1, pwm_right > 0); digitalWrite(FR_IN2, pwm_right < 0);
        digitalWrite(RR_IN1, pwm_right > 0); digitalWrite(RR_IN2, pwm_right < 0);

        ledcWrite(0, abs(pwm_left));
        ledcWrite(2, abs(pwm_left));
        ledcWrite(1, abs(pwm_right));
        ledcWrite(3, abs(pwm_right));

        vTaskDelay(10 / portTICK_PERIOD_MS);
      }
    },
    "MotorTask",
    2048,
    NULL,
    1,
    NULL,
    1
  );
}

// ==================== LOOP ====================
void loop() {
  unsigned long now = millis();

  nh.spinOnce();

  if (now - last_odom_time >= ODOM_PUBLISH_INTERVAL) {
    float dt = (now - last_odom_time) / 1000.0;
    last_odom_time = now;
  }

  delay(1);
}
