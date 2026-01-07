#include <WiFi.h>
#include <WiFiClient.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <Wire.h>
#include <MPU9250.h>

// ==================== ROS WiFi Hardware ====================
class RosWiFiHardware {
public:
  WiFiClient* client;
  IPAddress server;
  uint16_t port;
  unsigned long baud_; 
  RosWiFiHardware() : client(nullptr), server(), port(0), baud_(0) {}
  RosWiFiHardware(WiFiClient* c, IPAddress s, uint16_t p) : client(c), server(s), port(p), baud_(0) {}
  void setConnection(IPAddress s, uint16_t p) { server = s; port = p; }
  void init() { if (client && !client->connected()) client->connect(server, port); }
  int read() { return (client && client->connected() && client->available()) ? client->read() : -1; }
  void write(uint8_t* data, int length) { if (client && client->connected()) client->write(data, length); }
  unsigned long time() { return millis(); }
};

// ==================== SETTINGS ====================
#define ROS_SERIAL_BUFFER_SIZE 1024
const char* ssid = "ShellBack";
const char* password = "hhmo@1974";
IPAddress server(192, 168, 1, 16); 
const uint16_t serverPort = 11411;

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

#define IMU_SDA 21
#define IMU_SCL 22

// ==================== GLOBALS ====================
WiFiClient client;
RosWiFiHardware ros_wifi_hw(&client, server, serverPort);
ros::NodeHandle_<RosWiFiHardware, 25, 25, ROS_SERIAL_BUFFER_SIZE, ROS_SERIAL_BUFFER_SIZE> nh;

volatile long encoder_fl=0, encoder_fr=0, encoder_rl=0, encoder_rr=0;
long prev_encoder_fl=0, prev_encoder_fr=0, prev_encoder_rl=0, prev_encoder_rr=0;
int motor_speeds[4] = {0,0,0,0}; 

float target_vel_fl=0, target_vel_fr=0, target_vel_rl=0, target_vel_rr=0;
float current_vel_fl=0, current_vel_fr=0, current_vel_rl=0, current_vel_rr=0;
float error_sum_fl=0, error_sum_fr=0, error_sum_rl=0, error_sum_rr=0;
float prev_error_fl=0, prev_error_fr=0, prev_error_rl=0, prev_error_rr=0;

float odom_x = 0, odom_y = 0, odom_theta = 0;
MPU9250 mpu;

// TWEAKED PID GAINS FOR STABILITY
float kp = 5.0, ki = 2.0, kd = 0.1; 
const float WHEEL_RADIUS = 0.0325;
const float TRACK_WIDTH = 0.26;
const int TICKS_PER_REV = 4900; 
unsigned long last_control_time=0, last_cmd_time=0, last_imu_time=0;

nav_msgs::Odometry odom_msg;
sensor_msgs::Imu imu_msg;
ros::Publisher odom_pub("wheel_odom", &odom_msg);
ros::Publisher imu_pub("imu/data", &imu_msg);

// ==================== ISRs ====================
void IRAM_ATTR isrFL(){ if(digitalRead(ENC_FL_A)==digitalRead(ENC_FL_B)) encoder_fl--; else encoder_fl++; }
void IRAM_ATTR isrFR(){ if(digitalRead(ENC_FR_A)==digitalRead(ENC_FR_B)) encoder_fr--; else encoder_fr++; }
void IRAM_ATTR isrRL(){ if(digitalRead(ENC_RL_A)==digitalRead(ENC_RL_B)) encoder_rl--; else encoder_rl++; }
void IRAM_ATTR isrRR(){ if(digitalRead(ENC_RR_A)==digitalRead(ENC_RR_B)) encoder_rr--; else encoder_rr++; }

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
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  }

  // Store PWM value for software PWM (ALL motors use software PWM)
  motor_speeds[motor] = pwm_value;
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

  odom_msg.header.stamp = nh.now();
  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;
  odom_msg.pose.pose.orientation.z = sin(odom_theta * 0.5);
  odom_msg.pose.pose.orientation.w = cos(odom_theta * 0.5);
  odom_msg.twist.twist.linear.x = v_linear;
  odom_msg.twist.twist.angular.z = v_angular;
  odom_pub.publish(&odom_msg);
}

void controlLoop() {
  float dt = 0.02; // Increased dt to 20ms for better stability
  float ticks_to_m = (2.0 * PI * WHEEL_RADIUS) / TICKS_PER_REV;
  
  // Read encoder values atomically to avoid race conditions
  long enc_fl, enc_fr, enc_rl, enc_rr;
  noInterrupts();
  enc_fl = encoder_fl; enc_fr = encoder_fr;
  enc_rl = encoder_rl; enc_rr = encoder_rr;
  interrupts();
  
  current_vel_fl = (enc_fl - prev_encoder_fl) * ticks_to_m / dt;
  current_vel_fr = (enc_fr - prev_encoder_fr) * ticks_to_m / dt;
  current_vel_rl = (enc_rl - prev_encoder_rl) * ticks_to_m / dt;
  current_vel_rr = (enc_rr - prev_encoder_rr) * ticks_to_m / dt;

  prev_encoder_fl = enc_fl; prev_encoder_fr = enc_fr;
  prev_encoder_rl = enc_rl; prev_encoder_rr = enc_rr;

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

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Elderly Bot ESP32 Starting...");
  
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while(WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");

  // All motor pins as OUTPUT (software PWM, no hardware PWM setup)
  int outPins[] = {MOTOR_FL_IN1, MOTOR_FL_IN2, MOTOR_FL_PWM, MOTOR_FR_IN1, MOTOR_FR_IN2, MOTOR_FR_PWM,
                   MOTOR_RL_IN1, MOTOR_RL_IN2, MOTOR_RL_PWM, MOTOR_RR_IN1, MOTOR_RR_IN2, MOTOR_RR_PWM};
  for(int p : outPins) pinMode(p, OUTPUT);
  
  // Initialize all PWM pins to LOW (software PWM, no hardware PWM channels needed)
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

  // Initialize I2C for IMU
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000); // 400kHz I2C clock
  
  // Initialize MPU9250
  Serial.print("Initializing MPU9250...");
  if (mpu.begin()) {
    Serial.println(" OK");
    mpu.calibrateAccelGyro();
    Serial.println("IMU calibrated!");
  } else {
    Serial.println(" FAILED - IMU not detected");
  }

  *nh.getHardware() = ros_wifi_hw;
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(odom_pub);
  nh.advertise(imu_pub);
  
  // Initialize odometry message frame_id
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_footprint";
  
  // Initialize IMU message frame_id
  imu_msg.header.frame_id = "imu_link";
  
  Serial.println("ROS node initialized!");
  Serial.println("Elderly Bot Ready!");
}

void publishIMU() {
  if (mpu.update()) {
    imu_msg.header.stamp = nh.now();
    
    // Angular velocity (rad/s) - gyroscope data
    imu_msg.angular_velocity.x = mpu.getGyroX();
    imu_msg.angular_velocity.y = mpu.getGyroY();
    imu_msg.angular_velocity.z = mpu.getGyroZ();
    
    // Linear acceleration (m/s²) - accelerometer data (convert from g to m/s²)
    imu_msg.linear_acceleration.x = mpu.getAccX() * 9.80665;
    imu_msg.linear_acceleration.y = mpu.getAccY() * 9.80665;
    imu_msg.linear_acceleration.z = mpu.getAccZ() * 9.80665;
    
    // Set covariance matrices (diagonal)
    imu_msg.angular_velocity_covariance[0] = 0.02; // x
    imu_msg.angular_velocity_covariance[4] = 0.02; // y
    imu_msg.angular_velocity_covariance[8] = 0.02; // z
    
    imu_msg.linear_acceleration_covariance[0] = 0.04; // x
    imu_msg.linear_acceleration_covariance[4] = 0.04; // y
    imu_msg.linear_acceleration_covariance[8] = 0.04; // z
    
    imu_pub.publish(&imu_msg);
  }
}

void loop() {
  software_pwm_loop();
  // Safety timeout: stop motors if no command received for 500ms
  unsigned long current_time = millis();
  if((current_time - last_cmd_time > 500) || (current_time < last_cmd_time)) { // Handle millis() overflow
    target_vel_fl = target_vel_fr = target_vel_rl = target_vel_rr = 0;
  }
  if(millis() - last_control_time >= 20) { // Slower loop for stability
    controlLoop();
    last_control_time = millis();
  }
  // Publish IMU data at ~50Hz (every 20ms)
  if(millis() - last_imu_time >= 20) {
    publishIMU();
    last_imu_time = millis();
  }
  nh.spinOnce();
}