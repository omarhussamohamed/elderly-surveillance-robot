/*
 * ESP32 Firmware WITHOUT IMU Support - WiFi Version
 * * STRUCTURE: Uses WiFi structure from source (ShellBack)
 * LOGIC: Uses efficient loop and Hardware PWM from target
 */

#include <WiFi.h>
#include <WiFiClient.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// ==================== ROS WiFi Hardware (FROM SOURCE) ====================
class RosWiFiHardware {
public:
  WiFiClient* client;
  IPAddress server;
  uint16_t port;
  unsigned long baud_;

  RosWiFiHardware() : client(nullptr), server(), port(0), baud_(0) {}
  RosWiFiHardware(WiFiClient* c, IPAddress s, uint16_t p) : client(c), server(s), port(p), baud_(0) {}

  void setConnection(IPAddress s, uint16_t p) { server = s; port = p; }
  
  void init() { 
    if (client && !client->connected()) {
      client->connect(server, port);
    }
  }

  int read() { 
    return (client && client->connected() && client->available()) ? client->read() : -1; 
  }

  void write(uint8_t* data, int length) { 
    if (client && client->connected()) {
      client->write(data, length); 
    }
  }

  unsigned long time() { return millis(); }

  // Added for compatibility with the Target's loop logic
  bool connected() { return (client && client->connected()); }
};

// ==================== WiFi CONFIGURATION (FROM SOURCE) ====================
const char* ssid = "ShellBack";
const char* password = "hhmo@1974";
IPAddress server(192, 168, 1, 16); 
const uint16_t serverPort = 11411;

// ==================== SETTINGS (FROM TARGET) ====================
// Optimized timing intervals (milliseconds)
#define CONTROL_LOOP_INTERVAL 20   // 50Hz control loop (20ms)
#define ODOM_PUBLISH_INTERVAL 100  // 10Hz odometry (100ms)
#define SAFETY_TIMEOUT 800         // Stop motors if no cmd_vel for 800ms
#define ROS_SERIAL_BUFFER_SIZE 1024 // From Source

// Hardware PWM settings
#define PWM_FREQUENCY 5000         // 5kHz PWM frequency
#define PWM_RESOLUTION 8           // 8-bit resolution (0-255)
#define PWM_CHANNEL_FL 0           // LEDC channel for FL motor
#define PWM_CHANNEL_FR 1           // LEDC channel for FR motor
#define PWM_CHANNEL_RL 2           // LEDC channel for RL motor
#define PWM_CHANNEL_RR 3           // LEDC channel for RR motor

#define MIN_PWM 45             // Minimum power to overcome gearbox friction
#define MAX_PWM 255            // Maximum 8-bit PWM value

// ==================== PINOUT (FROM TARGET) ====================
#define MOTOR_FL_PWM 13
#define MOTOR_FL_IN1 12
#define MOTOR_FL_IN2 14
#define MOTOR_FR_PWM 27
#define MOTOR_FR_IN1 26
#define MOTOR_FR_IN2 25

#define MOTOR_RL_PWM 21  
#define MOTOR_RL_IN1 32
#define MOTOR_RL_IN2 15
#define MOTOR_RR_PWM 22 
#define MOTOR_RR_IN1 16
#define MOTOR_RR_IN2 17

// Encoders
#define ENC_FL_A 34
#define ENC_FL_B 35
#define ENC_FR_A 36
#define ENC_FR_B 39
#define ENC_RL_A 18
#define ENC_RL_B 19
#define ENC_RR_A 23
#define ENC_RR_B 5

// ==================== GLOBALS ====================
WiFiClient client;
RosWiFiHardware ros_wifi_hw(&client, server, serverPort);

// Using Source's NodeHandle sizing for stability
ros::NodeHandle_<RosWiFiHardware, 25, 25, ROS_SERIAL_BUFFER_SIZE, ROS_SERIAL_BUFFER_SIZE> nh;

volatile long encoder_fl = 0, encoder_fr = 0, encoder_rl = 0, encoder_rr = 0;
long prev_encoder_fl = 0, prev_encoder_fr = 0, prev_encoder_rl = 0, prev_encoder_rr = 0;
int motor_speeds[4] = {0, 0, 0, 0};

float target_vel_fl = 0, target_vel_fr = 0, target_vel_rl = 0, target_vel_rr = 0;
float current_vel_fl = 0, current_vel_fr = 0, current_vel_rl = 0, current_vel_rr = 0;
float error_sum_fl = 0, error_sum_fr = 0, error_sum_rl = 0, error_sum_rr = 0;
float prev_error_fl = 0, prev_error_fr = 0, prev_error_rl = 0, prev_error_rr = 0;

float odom_x = 0, odom_y = 0, odom_theta = 0;

// TWEAKED PID GAINS FOR STABILITY
float kp = 4.5, ki = 1.5, kd = 0.05;
const float WHEEL_RADIUS = 0.0325;
const float TRACK_WIDTH = 0.26;
const int TICKS_PER_REV = 4900;

unsigned long last_control_time = 0, last_cmd_time = 0, last_odom_time = 0, last_ros_connect = 0;

nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("wheel_odom", &odom_msg);

// ==================== ISRs ====================
void IRAM_ATTR isrFL() {
  digitalRead(ENC_FL_A) == digitalRead(ENC_FL_B) ? encoder_fl-- : encoder_fl++;
}
void IRAM_ATTR isrFR() {
  digitalRead(ENC_FR_A) == digitalRead(ENC_FR_B) ? encoder_fr-- : encoder_fr++;
}
void IRAM_ATTR isrRL() {
  digitalRead(ENC_RL_A) == digitalRead(ENC_RL_B) ? encoder_rl-- : encoder_rl++;
}
void IRAM_ATTR isrRR() {
  digitalRead(ENC_RR_A) == digitalRead(ENC_RR_B) ? encoder_rr-- : encoder_rr++;
}

// ==================== IMPROVED MOTOR LOGIC ====================
void setMotorSpeed(int motor, int pwm_value) {
  // 1. Identify Pins
  int in1, in2, pwm_chan;
  switch (motor) {
    case 0: in1 = MOTOR_FL_IN1; in2 = MOTOR_FL_IN2; pwm_chan = PWM_CHANNEL_FL; break;
    case 1: in1 = MOTOR_FR_IN1; in2 = MOTOR_FR_IN2; pwm_chan = PWM_CHANNEL_FR; break;
    case 2: in1 = MOTOR_RL_IN1; in2 = MOTOR_RL_IN2; pwm_chan = PWM_CHANNEL_RL; break;
    case 3: in1 = MOTOR_RR_IN1; in2 = MOTOR_RR_IN2; pwm_chan = PWM_CHANNEL_RR; break;
    default: return;
  }

  // 2. Deadzone Compensation: Helps motors move at low cmd_vel
  if (pwm_value != 0 && abs(pwm_value) < MIN_PWM) {
    pwm_value = (pwm_value > 0) ? MIN_PWM : -MIN_PWM;
  }

  // 3. Apply Direction (Standardized)
  if (pwm_value > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (pwm_value < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  ledcWrite(pwm_chan, abs(pwm_value));
}

int computePID(float target, float current, float &error_sum, float &prev_error, float dt) {
  if (abs(target) < 0.01) {
    error_sum = 0;
    prev_error = 0;
    return 0;
  }

  float error = target - current;
  error_sum += error * dt;
  error_sum = constrain(error_sum, -50, 50);

  float derivative = (dt > 0) ? (error - prev_error) / dt : 0;
  prev_error = error;

  return constrain((int)(kp * error + ki * error_sum + kd * derivative), -255, 255);
}

void updateOdometry(float dt) {
  if (dt <= 0) return;

  float v_left = (current_vel_fl + current_vel_rl) / 2.0;
  float v_right = (current_vel_fr + current_vel_rr) / 2.0;
  float v_linear = (v_left + v_right) / 2.0;
  float v_angular = (v_right - v_left) / TRACK_WIDTH;

  odom_theta += v_angular * dt;
  odom_x += v_linear * cos(odom_theta) * dt;
  odom_y += v_linear * sin(odom_theta) * dt;

  while (odom_theta > PI) odom_theta -= 2 * PI;
  while (odom_theta < -PI) odom_theta += 2 * PI;
}

void publishOdometry() {
  odom_msg.header.stamp = nh.now();
  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;
  odom_msg.pose.pose.orientation.z = sin(odom_theta * 0.5);
  odom_msg.pose.pose.orientation.w = cos(odom_theta * 0.5);
  
  float v_left = (current_vel_fl + current_vel_rl) / 2.0;
  float v_right = (current_vel_fr + current_vel_rr) / 2.0;
  float v_linear = (v_left + v_right) / 2.0;
  float v_angular = (v_right - v_left) / TRACK_WIDTH;
  
  odom_msg.twist.twist.linear.x = v_linear;
  odom_msg.twist.twist.angular.z = v_angular;
  
  odom_pub.publish(&odom_msg);
}

void controlLoop() {
  float dt = CONTROL_LOOP_INTERVAL / 1000.0;
  float ticks_to_m = (2.0 * PI * WHEEL_RADIUS) / TICKS_PER_REV;

  noInterrupts();
  long enc_fl = encoder_fl, enc_fr = encoder_fr;
  long enc_rl = encoder_rl, enc_rr = encoder_rr;
  interrupts();

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

void cmdVelCallback(const geometry_msgs::Twist& cmd_vel) {
  last_cmd_time = millis();
  float v = cmd_vel.linear.x;
  float w = cmd_vel.angular.z;
  target_vel_fl = target_vel_rl = v - w * (TRACK_WIDTH / 2.0);
  target_vel_fr = target_vel_rr = v + w * (TRACK_WIDTH / 2.0);
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmdVelCallback);

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Elderly Bot ESP32 Starting...");
  
  // Connect to WiFi (Using Source Logic)
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while(WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");

  // Initialize motor control pins (Hardware PWM)
  int outPins[] = {MOTOR_FL_IN1, MOTOR_FL_IN2, MOTOR_FL_PWM,
                   MOTOR_FR_IN1, MOTOR_FR_IN2, MOTOR_FR_PWM,
                   MOTOR_RL_IN1, MOTOR_RL_IN2, MOTOR_RL_PWM,
                   MOTOR_RR_IN1, MOTOR_RR_IN2, MOTOR_RR_PWM};
  for (int i = 0; i < 12; i++) {
    pinMode(outPins[i], OUTPUT);
    digitalWrite(outPins[i], LOW);
  }

  // Initialize encoder pins
  pinMode(ENC_FL_A, INPUT); pinMode(ENC_FL_B, INPUT);
  pinMode(ENC_FR_A, INPUT); pinMode(ENC_FR_B, INPUT);
  pinMode(ENC_RL_A, INPUT_PULLUP); pinMode(ENC_RL_B, INPUT_PULLUP);
  pinMode(ENC_RR_A, INPUT_PULLUP); pinMode(ENC_RR_B, INPUT_PULLUP);

  // Setup Hardware PWM channels
  ledcSetup(PWM_CHANNEL_FL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_FR, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_RL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_RR, PWM_FREQUENCY, PWM_RESOLUTION);

  ledcAttachPin(MOTOR_FL_PWM, PWM_CHANNEL_FL);
  ledcAttachPin(MOTOR_FR_PWM, PWM_CHANNEL_FR);
  ledcAttachPin(MOTOR_RL_PWM, PWM_CHANNEL_RL);
  ledcAttachPin(MOTOR_RR_PWM, PWM_CHANNEL_RR);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_FL_A), isrFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_A), isrFR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RL_A), isrRL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RR_A), isrRR, CHANGE);

  // ROS setup (Using Source Logic)
  *nh.getHardware() = ros_wifi_hw;
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(odom_pub);

  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_footprint";

  Serial.println("ROS node initialized!");
}

// ==================== LOOP ====================
void loop() {
  unsigned long now = millis();

  // Connection management
  if (!ros_wifi_hw.connected()) {
    // Reconnection logic handled by nh.spinOnce in next iteration or simple retry
    // The source code relied on init() inside setup, but nh.spinOnce handles reconnects
    // if the hardware class init is robust.
    if (now - last_ros_connect > 2000) {
       ros_wifi_hw.init();
       last_ros_connect = now;
    }
  }

  nh.spinOnce();

  // Control loop (50Hz = every 20ms)
  if (now - last_control_time >= CONTROL_LOOP_INTERVAL) {
    controlLoop();
    last_control_time = now;
  }

  // Safety timeout
  if (now - last_cmd_time > SAFETY_TIMEOUT) {
    target_vel_fl = target_vel_fr = target_vel_rl = target_vel_rr = 0;
  }

  // Publish odometry (10Hz = every 100ms)
  if (now - last_odom_time >= ODOM_PUBLISH_INTERVAL) {
    publishOdometry();
    last_odom_time = now;
  }
}