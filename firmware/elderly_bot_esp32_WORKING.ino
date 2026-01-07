#include <WiFi.h>
#include <WiFiClient.h>

// Minimal custom hardware class for rosserial over WiFi
class RosWiFiHardware {
public:
  WiFiClient* client;
  IPAddress server;
  uint16_t port;
  unsigned long baud_; // not used, for compatibility

  // Default constructor (required by ros::NodeHandle_)
  RosWiFiHardware() : client(nullptr), server(), port(0), baud_(0) {}

  RosWiFiHardware(WiFiClient* c, IPAddress s, uint16_t p) : client(c), server(s), port(p), baud_(0) {}

  void setConnection(IPAddress s, uint16_t p) {
    server = s;
    port = p;
  }

  void init() {
    if (client && !client->connected()) {
      client->connect(server, port);
    }
  }

  int read() {
    if (client && client->connected() && client->available()) {
      return client->read();
    }
    return -1;
  }

  void write(uint8_t* data, int length) {
    if (client && client->connected()) {
      client->write(data, length);
    }
  }

  unsigned long time() {
    return millis();
  }
};
/*
 * Elderly Bot ESP32 Firmware - WORKING VERSION
 * Tested with ESP32 Arduino Core 2.0.17 + ROS Melodic
 * 
 * CRITICAL: Must use ESP32 Arduino Core 2.0.17 (NOT 3.x)
 */


#include <WiFi.h>
#include <WiFiClient.h>

#define ROS_SERIAL_BUFFER_SIZE 1024

const char* ssid = "ShellBack";
const char* password = "hhmo@1974";
IPAddress server(192,168,1,16); // Jetson Nano's IP
const uint16_t serverPort = 11411; // Default rosserial TCP port

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <Wire.h>
#include <MPU9250.h>

// ==================== HARDWARE PINS ====================
// ...existing code...

// Motor Driver L298N #1
#define MOTOR_FL_PWM 32
#define MOTOR_FL_IN1 14
#define MOTOR_FL_IN2 33
#define MOTOR_FR_PWM 25
#define MOTOR_FR_IN1 26
#define MOTOR_FR_IN2 27

// Motor Driver L298N #2
#define MOTOR_RL_PWM 4
#define MOTOR_RL_IN1 5
#define MOTOR_RL_IN2 16
#define MOTOR_RR_PWM 17
#define MOTOR_RR_IN1 18
#define MOTOR_RR_IN2 19

// Encoders
#define ENC_FL_A 34
#define ENC_FL_B 35
#define ENC_FR_A 36
#define ENC_FR_B 39
#define ENC_RL_A 13
#define ENC_RL_B 12
#define ENC_RR_A 15
#define ENC_RR_B 2

// IMU
#define IMU_SDA 21
#define IMU_SCL 22

// PWM
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8
#define PWM_MAX 255
#define PWM_CH_FL 0
#define PWM_CH_FR 1
#define PWM_CH_RL 2
#define PWM_CH_RR 3

// ==================== ROBOT PARAMETERS ====================

const float WHEEL_RADIUS = 0.0325;
const float TRACK_WIDTH = 0.26;
const int TICKS_PER_REV = 3960;
const float MAX_LINEAR_VEL = 0.25;
const float MAX_ANGULAR_VEL = 1.0;
const unsigned long CONTROL_PERIOD_MS = 10;
const unsigned long CMD_TIMEOUT_MS = 500;

float kp = 25.0;
float ki = 5.0;
float kd = 0.5;

bool invert_fl = false;
bool invert_fr = true;
bool invert_rl = false;
bool invert_rr = true;

// ==================== GLOBAL VARIABLES ====================


WiFiClient client;
RosWiFiHardware ros_wifi_hw(&client, server, serverPort);
ros::NodeHandle_<RosWiFiHardware, 25, 25, ROS_SERIAL_BUFFER_SIZE, ROS_SERIAL_BUFFER_SIZE> nh;
volatile long encoder_fl = 0;
volatile long encoder_fr = 0;
volatile long encoder_rl = 0;
volatile long encoder_rr = 0;

long prev_encoder_fl = 0;
long prev_encoder_fr = 0;
long prev_encoder_rl = 0;
long prev_encoder_rr = 0;

float target_vel_fl = 0.0;
float target_vel_fr = 0.0;
float target_vel_rl = 0.0;
float target_vel_rr = 0.0;

float current_vel_fl = 0.0;
float current_vel_fr = 0.0;
float current_vel_rl = 0.0;
float current_vel_rr = 0.0;

float error_sum_fl = 0.0;
float error_sum_fr = 0.0;
float error_sum_rl = 0.0;
float error_sum_rr = 0.0;

float prev_error_fl = 0.0;
float prev_error_fr = 0.0;
float prev_error_rl = 0.0;
float prev_error_rr = 0.0;

unsigned long last_control_time = 0;
unsigned long last_cmd_time = 0;
unsigned long last_imu_time = 0;

MPU9250 mpu;
const unsigned long IMU_PERIOD_MS = 10;

float odom_x = 0.0;
float odom_y = 0.0;
float odom_theta = 0.0;

nav_msgs::Odometry odom_msg;
sensor_msgs::Imu imu_msg;

// ==================== FUNCTION PROTOTYPES ====================

void cmdVelCallback(const geometry_msgs::Twist& cmd_vel);
void computeWheelVelocities(float linear_x, float angular_z);
void updateOdometry(float dt);
void controlLoop();
void safeStop();
void setupMotors();
void setupEncoders();
void setupIMU();
void publishIMU();
int computePID(float target, float current, float &error_sum, float &prev_error, float dt);
void setMotorSpeed(int motor, int pwm_value);

// ==================== ROS PUBLISHERS & SUBSCRIBERS ====================

ros::Publisher odom_pub("wheel_odom", &odom_msg);
ros::Publisher imu_pub("imu/data", &imu_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmdVelCallback);

// ==================== INTERRUPT SERVICE ROUTINES ====================

void IRAM_ATTR encoderFL_ISR() {
  if (digitalRead(ENC_FL_B)) encoder_fl++;
  else encoder_fl--;
}

void IRAM_ATTR encoderFR_ISR() {
  if (digitalRead(ENC_FR_B)) encoder_fr++;
  else encoder_fr--;
}

void IRAM_ATTR encoderRL_ISR() {
  if (digitalRead(ENC_RL_B)) encoder_rl++;
  else encoder_rl--;
}

void IRAM_ATTR encoderRR_ISR() {
  if (digitalRead(ENC_RR_B)) encoder_rr++;
  else encoder_rr--;
}

// ==================== SETUP ====================

void setup() {
  // Serial MUST be initialized first, with delay
  Serial.begin(115200);
  delay(2000);  // CRITICAL: Wait for serial to stabilize
  
  Serial.println("\n\n========================================");
  Serial.println("Elderly Bot ESP32 Firmware");
  Serial.println("Core: 2.0.17 | ROS: Melodic");
  Serial.println("========================================\n");
  
  // Initialize hardware BEFORE ROS
  Serial.println("[1/4] Initializing motors...");
  setupMotors();
  
  Serial.println("[2/4] Initializing encoders...");
  setupEncoders();
  
  Serial.println("[3/4] Initializing IMU...");
  setupIMU();
  
  Serial.println("[4/4] Initializing ROS node...");
  delay(1000);  // CRITICAL: Delay before ROS init
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected, IP: ");
  Serial.println(WiFi.localIP());

  *nh.getHardware() = ros_wifi_hw;
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  
  // Longer delay after initNode
  delay(500);
  
  nh.advertise(odom_pub);
  nh.advertise(imu_pub);
  nh.subscribe(cmd_vel_sub);
  
  Serial.println("\n[READY] Waiting for rosserial TCP connection...");
  // Removed USB serial print statement
  
  // Initialize timing
  last_control_time = millis();
  last_cmd_time = millis();
  last_imu_time = millis();
  
  // Initialize messages
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_footprint";
  imu_msg.header.frame_id = "imu_link";
  imu_msg.orientation_covariance[0] = -1.0;
}

// ==================== MAIN LOOP ====================

void loop() {
  unsigned long current_time = millis();
  
  // Check cmd_vel timeout
  if (current_time - last_cmd_time > CMD_TIMEOUT_MS) {
    safeStop();
  }
  
  // Control loop at 100 Hz
  if (current_time - last_control_time >= CONTROL_PERIOD_MS) {
    controlLoop();
    last_control_time = current_time;
  }
  
  // IMU at 100 Hz
  if (current_time - last_imu_time >= IMU_PERIOD_MS) {
    publishIMU();
    last_imu_time = current_time;
  }
  
  // ROS communication
  nh.spinOnce();
  delay(1);
}

// ==================== CALLBACKS ====================

void cmdVelCallback(const geometry_msgs::Twist& cmd_vel) {
  last_cmd_time = millis();
  float linear_x = constrain(cmd_vel.linear.x, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
  float angular_z = constrain(cmd_vel.angular.z, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
  computeWheelVelocities(linear_x, angular_z);
}

// ==================== KINEMATICS ====================

void computeWheelVelocities(float linear_x, float angular_z) {
  float v_left = linear_x - angular_z * (TRACK_WIDTH / 2.0);
  float v_right = linear_x + angular_z * (TRACK_WIDTH / 2.0);
  
  target_vel_fl = v_left;
  target_vel_rl = v_left;
  target_vel_fr = v_right;
  target_vel_rr = v_right;
}

// ==================== CONTROL ====================

void controlLoop() {
  float dt = CONTROL_PERIOD_MS / 1000.0;
  
  long delta_fl = encoder_fl - prev_encoder_fl;
  long delta_fr = encoder_fr - prev_encoder_fr;
  long delta_rl = encoder_rl - prev_encoder_rl;
  long delta_rr = encoder_rr - prev_encoder_rr;
  
  prev_encoder_fl = encoder_fl;
  prev_encoder_fr = encoder_fr;
  prev_encoder_rl = encoder_rl;
  prev_encoder_rr = encoder_rr;
  
  float ticks_to_meters = (2.0 * PI * WHEEL_RADIUS) / TICKS_PER_REV;
  current_vel_fl = (delta_fl * ticks_to_meters) / dt;
  current_vel_fr = (delta_fr * ticks_to_meters) / dt;
  current_vel_rl = (delta_rl * ticks_to_meters) / dt;
  current_vel_rr = (delta_rr * ticks_to_meters) / dt;
  
  int pwm_fl = computePID(target_vel_fl, current_vel_fl, error_sum_fl, prev_error_fl, dt);
  int pwm_fr = computePID(target_vel_fr, current_vel_fr, error_sum_fr, prev_error_fr, dt);
  int pwm_rl = computePID(target_vel_rl, current_vel_rl, error_sum_rl, prev_error_rl, dt);
  int pwm_rr = computePID(target_vel_rr, current_vel_rr, error_sum_rr, prev_error_rr, dt);
  
  setMotorSpeed(0, pwm_fl);
  setMotorSpeed(1, pwm_fr);
  setMotorSpeed(2, pwm_rl);
  setMotorSpeed(3, pwm_rr);
  
  updateOdometry(dt);
}

int computePID(float target, float current, float &error_sum, float &prev_error, float dt) {
  float error = target - current;
  float p_term = kp * error;
  
  error_sum += error * dt;
  error_sum = constrain(error_sum, -10.0, 10.0);
  float i_term = ki * error_sum;
  
  float d_term = 0.0;
  if (dt > 0) {
    d_term = kd * (error - prev_error) / dt;
  }
  prev_error = error;
  
  float output = p_term + i_term + d_term;
  return constrain((int)output, -PWM_MAX, PWM_MAX);
}

void setMotorSpeed(int motor, int pwm_value) {
  bool invert = false;
  int pwm_channel = 0;
  int in1_pin = 0;
  int in2_pin = 0;
  
  switch(motor) {
    case 0: invert = invert_fl; pwm_channel = PWM_CH_FL; in1_pin = MOTOR_FL_IN1; in2_pin = MOTOR_FL_IN2; break;
    case 1: invert = invert_fr; pwm_channel = PWM_CH_FR; in1_pin = MOTOR_FR_IN1; in2_pin = MOTOR_FR_IN2; break;
    case 2: invert = invert_rl; pwm_channel = PWM_CH_RL; in1_pin = MOTOR_RL_IN1; in2_pin = MOTOR_RL_IN2; break;
    case 3: invert = invert_rr; pwm_channel = PWM_CH_RR; in1_pin = MOTOR_RR_IN1; in2_pin = MOTOR_RR_IN2; break;
  }
  
  if (invert) pwm_value = -pwm_value;
  
  if (pwm_value > 0) {
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
    ledcWrite(pwm_channel, abs(pwm_value));
  } else if (pwm_value < 0) {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
    ledcWrite(pwm_channel, abs(pwm_value));
  } else {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, LOW);
    ledcWrite(pwm_channel, 0);
  }
}

void safeStop() {
  target_vel_fl = target_vel_fr = target_vel_rl = target_vel_rr = 0.0;
  error_sum_fl = error_sum_fr = error_sum_rl = error_sum_rr = 0.0;
  
  ledcWrite(PWM_CH_FL, 0);
  ledcWrite(PWM_CH_FR, 0);
  ledcWrite(PWM_CH_RL, 0);
  ledcWrite(PWM_CH_RR, 0);
  
  digitalWrite(MOTOR_FL_IN1, LOW); digitalWrite(MOTOR_FL_IN2, LOW);
  digitalWrite(MOTOR_FR_IN1, LOW); digitalWrite(MOTOR_FR_IN2, LOW);
  digitalWrite(MOTOR_RL_IN1, LOW); digitalWrite(MOTOR_RL_IN2, LOW);
  digitalWrite(MOTOR_RR_IN1, LOW); digitalWrite(MOTOR_RR_IN2, LOW);
}

// ==================== ODOMETRY ====================

void updateOdometry(float dt) {
  float v_left = (current_vel_fl + current_vel_rl) / 2.0;
  float v_right = (current_vel_fr + current_vel_rr) / 2.0;
  float v_linear = (v_left + v_right) / 2.0;
  float v_angular = (v_right - v_left) / TRACK_WIDTH;
  
  odom_theta += v_angular * dt;
  odom_x += v_linear * cos(odom_theta) * dt;
  odom_y += v_linear * sin(odom_theta) * dt;
  
  while (odom_theta > PI) odom_theta -= 2.0 * PI;
  while (odom_theta < -PI) odom_theta += 2.0 * PI;
  
  odom_msg.header.stamp = nh.now();
  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;
  odom_msg.pose.pose.position.z = 0.0;
  
  float cy = cos(odom_theta * 0.5);
  float sy = sin(odom_theta * 0.5);
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sy;
  odom_msg.pose.pose.orientation.w = cy;
  
  odom_msg.twist.twist.linear.x = v_linear;
  odom_msg.twist.twist.angular.z = v_angular;
  
  odom_msg.pose.covariance[0] = 0.001;
  odom_msg.pose.covariance[7] = 0.001;
  odom_msg.pose.covariance[35] = 0.01;
  odom_msg.twist.covariance[0] = 0.001;
  odom_msg.twist.covariance[35] = 0.01;
  
  odom_pub.publish(&odom_msg);
}

// ==================== IMU ====================

void setupIMU() {
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);
  delay(100);
  
  if (!mpu.setup(0x68)) {
    Serial.println("[ERROR] MPU-9250 not found!");
    while(1) delay(1000);
  }
  
  Serial.println("Calibrating IMU (keep still)...");
  delay(2000);
  mpu.calibrateAccelGyro();
  Serial.println("IMU ready!");
}

void publishIMU() {
  if (mpu.update()) {
    imu_msg.header.stamp = nh.now();
    imu_msg.angular_velocity.x = mpu.getGyroX();
    imu_msg.angular_velocity.y = mpu.getGyroY();
    imu_msg.angular_velocity.z = mpu.getGyroZ();
    imu_msg.linear_acceleration.x = mpu.getAccX() * 9.80665;
    imu_msg.linear_acceleration.y = mpu.getAccY() * 9.80665;
    imu_msg.linear_acceleration.z = mpu.getAccZ() * 9.80665;
    imu_msg.angular_velocity_covariance[0] = 0.02;
    imu_msg.angular_velocity_covariance[4] = 0.02;
    imu_msg.angular_velocity_covariance[8] = 0.02;
    imu_msg.linear_acceleration_covariance[0] = 0.04;
    imu_msg.linear_acceleration_covariance[4] = 0.04;
    imu_msg.linear_acceleration_covariance[8] = 0.04;
    imu_pub.publish(&imu_msg);
  }
}

// ==================== HARDWARE SETUP ====================

void setupMotors() {
  pinMode(MOTOR_FL_IN1, OUTPUT); pinMode(MOTOR_FL_IN2, OUTPUT);
  pinMode(MOTOR_FR_IN1, OUTPUT); pinMode(MOTOR_FR_IN2, OUTPUT);
  pinMode(MOTOR_RL_IN1, OUTPUT); pinMode(MOTOR_RL_IN2, OUTPUT);
  pinMode(MOTOR_RR_IN1, OUTPUT); pinMode(MOTOR_RR_IN2, OUTPUT);
  
  ledcSetup(PWM_CH_FL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_FR, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_RL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_RR, PWM_FREQ, PWM_RESOLUTION);
  
  ledcAttachPin(MOTOR_FL_PWM, PWM_CH_FL);
  ledcAttachPin(MOTOR_FR_PWM, PWM_CH_FR);
  ledcAttachPin(MOTOR_RL_PWM, PWM_CH_RL);
  ledcAttachPin(MOTOR_RR_PWM, PWM_CH_RR);
  
  safeStop();
}

void setupEncoders() {
  pinMode(ENC_FL_A, INPUT); pinMode(ENC_FL_B, INPUT);
  pinMode(ENC_FR_A, INPUT); pinMode(ENC_FR_B, INPUT);
  pinMode(ENC_RL_A, INPUT_PULLUP); pinMode(ENC_RL_B, INPUT_PULLUP);
  pinMode(ENC_RR_A, INPUT_PULLUP); pinMode(ENC_RR_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENC_FL_A), encoderFL_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_A), encoderFR_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RL_A), encoderRL_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RR_A), encoderRR_ISR, CHANGE);
}

