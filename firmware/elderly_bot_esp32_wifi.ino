/*
 * ESP32 Firmware WITHOUT IMU Support - WiFi Version
 * 
 * Use this version if MPU9250 is connected directly to Jetson I2C.
 * This version uses WiFi for rosserial communication.
 * 
 * Connection method: Uses proven RosWiFiHardware from elderly_bot_esp32.ino
 * Efficiency: Simple loop-based timing (no FreeRTOS tasks overhead)
 * Data optimization: Reduced publish rates, minimal message sizes
 * 
 * To use:
 * 1. Update WiFi credentials below (SSID, PASSWORD, ROS_SERVER_IP)
 * 2. Upload to ESP32
 * 3. On Jetson, run: rosrun rosserial_python serial_node.py tcp
 */

#include <WiFi.h>
#include <WiFiClient.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// ==================== ROS WiFi Hardware (Proven from elderly_bot_esp32.ino) ====================
class RosWiFiHardware {
public:
  WiFiClient* client;
  IPAddress server;
  uint16_t port;

  RosWiFiHardware() : client(nullptr), server(), port(0) {}
  RosWiFiHardware(WiFiClient* c, IPAddress s, uint16_t p) : client(c), server(s), port(p) {}

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
    return (client && client->connected() && client->available()) ? client->read() : -1;
  }

  void write(uint8_t* data, int length) {
    if (client && client->connected()) {
      client->write(data, length);
    }
  }

  unsigned long time() {
    return millis();
  }

  bool connected() {
    return client && client->connected();
  }

  bool connect() {
    if (!client) return false;
    if (client->connected()) return true;

    Serial.print("Connecting to ROS server ");
    Serial.print(server);
    Serial.print(":");
    Serial.println(port);

    if (client->connect(server, port)) {
      Serial.println("ROS TCP connection successful!");
      return true;
    } else {
      Serial.println("ROS connection failed.");
      return false;
    }
  }
};

// ==================== WiFi CONFIGURATION ====================
// **CRITICAL**: Update these with your WiFi network credentials
const char* ssid = "ShellBack";
const char* password = "hhmo@1974";

// ROS Server IP (Jetson Nano IP address on your WiFi network)
IPAddress server(192, 168, 1, 100);  // Change to your Jetson's IP
const uint16_t serverPort = 11411;

// ==================== SETTINGS ====================
// Optimized timing intervals (milliseconds) - efficient, not resource-hungry
#define CONTROL_LOOP_INTERVAL 20   // 50Hz control loop (20ms)
#define ODOM_PUBLISH_INTERVAL 100  // 10Hz odometry (100ms) - reduced to save bandwidth
#define SAFETY_TIMEOUT 800         // Stop motors if no cmd_vel for 800ms

// Hardware PWM settings
#define PWM_FREQUENCY 5000         // 5kHz PWM frequency
#define PWM_RESOLUTION 8           // 8-bit resolution (0-255)
#define PWM_CHANNEL_FL 0           // LEDC channel for FL motor
#define PWM_CHANNEL_FR 1           // LEDC channel for FR motor
#define PWM_CHANNEL_RL 2           // LEDC channel for RL motor
#define PWM_CHANNEL_RR 3           // LEDC channel for RR motor

// ==================== PINOUT (FINAL VERIFIED) ====================
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
// *** PROVEN WiFi Hardware Setup (from elderly_bot_esp32.ino) ***
WiFiClient client;
RosWiFiHardware ros_wifi_hw(&client, server, serverPort);

// *** Increased buffer size to 1024 to avoid "message larger than buffer" errors ***
// Template: NodeHandle<Hardware, MaxSubscribers, MaxPublishers, InputBuffer, OutputBuffer>
ros::NodeHandle_<RosWiFiHardware, 10, 10, 512, 1024> nh;

volatile long encoder_fl = 0, encoder_fr = 0, encoder_rl = 0, encoder_rr = 0;
long prev_encoder_fl = 0, prev_encoder_fr = 0, prev_encoder_rl = 0, prev_encoder_rr = 0;
int motor_speeds[4] = {0, 0, 0, 0};

float target_vel_fl = 0, target_vel_fr = 0, target_vel_rl = 0, target_vel_rr = 0;
float current_vel_fl = 0, current_vel_fr = 0, current_vel_rl = 0, current_vel_rr = 0;
float error_sum_fl = 0, error_sum_fr = 0, error_sum_rl = 0, error_sum_rr = 0;
float prev_error_fl = 0, prev_error_fr = 0, prev_error_rl = 0, prev_error_rr = 0;

float odom_x = 0, odom_y = 0, odom_theta = 0;

// TWEAKED PID GAINS FOR STABILITY
float kp = 5.0, ki = 2.0, kd = 0.1;
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

// ==================== MOTOR LOGIC ====================
// Hardware PWM using ESP32 LEDC channels

void setMotorSpeed(int motor, int pwm_value) {
  pwm_value = constrain(pwm_value, -255, 255);

  int in1, in2;
  int pwm_channel;
  switch (motor) {
    case 0: in1 = MOTOR_FL_IN1; in2 = MOTOR_FL_IN2; pwm_channel = PWM_CHANNEL_FL; break;
    case 1: in1 = MOTOR_FR_IN1; in2 = MOTOR_FR_IN2; pwm_channel = PWM_CHANNEL_FR; break;
    case 2: in1 = MOTOR_RL_IN1; in2 = MOTOR_RL_IN2; pwm_channel = PWM_CHANNEL_RL; break;
    case 3: in1 = MOTOR_RR_IN1; in2 = MOTOR_RR_IN2; pwm_channel = PWM_CHANNEL_RR; break;
    default: return;
  }

  // Direction control
  if (pwm_value > 5) {
    digitalWrite(in1, (motor < 2 ? LOW : HIGH));
    digitalWrite(in2, (motor < 2 ? HIGH : LOW));
    ledcWrite(pwm_channel, abs(pwm_value));
  } else if (pwm_value < -5) {
    digitalWrite(in1, (motor < 2 ? HIGH : LOW));
    digitalWrite(in2, (motor < 2 ? LOW : HIGH));
    ledcWrite(pwm_channel, abs(pwm_value));
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(pwm_channel, 0);
    pwm_value = 0;
  }

  motor_speeds[motor] = abs(pwm_value);
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
  // Minimize message size - only set essential fields
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
  
  // Note: Covariance matrices are left at default (zeros) to minimize message size
  odom_pub.publish(&odom_msg);
}

void controlLoop() {
  float dt = CONTROL_LOOP_INTERVAL / 1000.0;  // Convert ms to seconds
  float ticks_to_m = (2.0 * PI * WHEEL_RADIUS) / TICKS_PER_REV;

  // Read encoder values atomically
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
  delay(2000);  // Allow Serial to stabilize
  Serial.println("\n\n=== Elderly Bot ESP32 WiFi Firmware (NO IMU) ===");

  // Initialize motor control pins
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

  // Connect to WiFi
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected! IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Signal strength (RSSI): ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");

  // ROS setup - CRITICAL: Use exact pattern from elderly_bot_esp32.ino
  *nh.getHardware() = ros_wifi_hw;
  nh.initNode();
  nh.advertise(odom_pub);
  nh.subscribe(cmd_vel_sub);

  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_footprint";

  Serial.println("\n=== Attempting to connect to ROS server ===");
  Serial.print("ROS Server IP: ");
  Serial.println(server);
  Serial.print("Port: ");
  Serial.println(serverPort);
  
  // Try to connect immediately
  if (ros_wifi_hw.connect()) {
    Serial.println("Connected to ROS server! Initializing ROS node...");
    delay(500);  // Allow connection to stabilize
  } else {
    Serial.println("WARNING: Could not connect to ROS server!");
    Serial.println("Please ensure:");
    Serial.println("  1. Jetson IP is correct (current: " + String(server[0]) + "." + String(server[1]) + "." + String(server[2]) + "." + String(server[3]) + ")");
    Serial.println("  2. ROS TCP server is running:");
    Serial.println("     rosrun rosserial_python serial_node.py tcp");
    Serial.println("  3. roscore is running");
    Serial.println("\nWill retry every 2 seconds...");
  }
}

// ==================== LOOP (Efficient - no FreeRTOS overhead) ====================
void loop() {
  unsigned long now = millis();

  // Reconnect to ROS if disconnected
  if (!ros_wifi_hw.connected()) {
    if (now - last_ros_connect > 2000) {
      Serial.print("Attempting to connect to ROS server... ");
      if (ros_wifi_hw.connect()) {
        Serial.println("Connected!");
        delay(500);  // Allow connection to stabilize
      } else {
        Serial.println("Failed. Will retry...");
      }
      last_ros_connect = now;
    }
  } else {
    // Call spinOnce to process incoming messages (cmd_vel)
    nh.spinOnce();

    // Control loop (50Hz = every 20ms)
    if (now - last_control_time >= CONTROL_LOOP_INTERVAL) {
      controlLoop();
      last_control_time = now;
    }

    // Safety timeout: stop motors if no command received
    if (now - last_cmd_time > SAFETY_TIMEOUT) {
      target_vel_fl = target_vel_fr = target_vel_rl = target_vel_rr = 0;
    }

    // Publish odometry (10Hz = every 100ms) - reduced rate to save bandwidth
    if (now - last_odom_time >= ODOM_PUBLISH_INTERVAL) {
      publishOdometry();
      last_odom_time = now;
    }
  }

  delay(1);  // Small delay to prevent watchdog issues
}
