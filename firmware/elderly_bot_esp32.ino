/*
 * ==================== ROS CONNECTION NOTES ====================
 * 
 * Connection History & Fixes (Critical - Do NOT change these parts lightly!)
 * 
 * Initial Issue:
 * - Code 1: Connected successfully to rosserial_python (TCP mode) on Jetson.
 * - Code 2: Failed to connect reliably on Jetson side despite improved motor control.
 * 
 * Root Cause:
 * - Code 2 used a different RosWiFiHardware implementation (client as member, no pointer).
 * - rosserial_ros_lib (ros_lib) on Arduino/ESP32 requires the hardware object to be assigned via 
 *   pointer using: *nh.getHardware() = ros_wifi_hw;
 * - Direct constructor passing or member-based client broke the internal rosserial protocol handling.
 * 
 * Additional Issue After Connection:
 * - "Message from device dropped: message larger than buffer" errors.
 * - Caused by default output buffer (512 bytes) being too small for nav_msgs/Odometry and sensor_msgs/Imu 
 *   (especially with full covariance fields).
 * 
 * Final Working Solution (Applied Here):
 * 1. Restored proven RosWiFiHardware from Code 1:
 *    - WiFiClient as external global object.
 *    - Hardware class takes pointer to client.
 *    - Assignment: *nh.getHardware() = ros_wifi_hw;  // <-- CRITICAL LINE
 * 2. Increased output buffer in NodeHandle:
 *    ros::NodeHandle_<..., 512, 1024> nh;  // Input 512, Output 1024
 * 3. Added automatic reconnection logic with serial feedback.
 * 
 * What NOT to Change:
 * - Do NOT pass hardware in NodeHandle constructor (e.g., nh(ros_wifi_hw)) → will NOT compile.
 * - Do NOT make WiFiClient a member of RosWiFiHardware → breaks rosserial compatibility.
 * - Do NOT reduce output buffer below 1024 → will cause dropped messages again.
 * - Keep *nh.getHardware() = ros_wifi_hw; and nh.initNode(); in setup() exactly as is.
 * 
 * This configuration gives:
 * - Reliable TCP connection to rosserial_python tcp on Jetson/PC
 * - No dropped messages
 * - Full compatibility with cmd_vel → odom/imu loop
 * 
 * Tested & Confirmed Working: January 2026
 */

 #include <WiFi.h>
 #include <WiFiClient.h>
 #include <ros.h>
 #include <geometry_msgs/Twist.h>
 #include <nav_msgs/Odometry.h>
 #include <sensor_msgs/Imu.h>
 #include <Wire.h>
 #include <MPU9250.h>
 
 // ==================== ROS WiFi Hardware (Proven) ====================
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
 
 // ==================== SETTINGS ====================
 const char* ssid = "ShellBack";
 const char* password = "hhmo@1974";
 IPAddress server(192, 168, 1, 16);
 const uint16_t serverPort = 11411;
 
 // ==================== PINOUT ====================
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
 
 // *** INCREASED BUFFER SIZE TO 1024 TO FIX "message larger than buffer" ***
 ros::NodeHandle_<RosWiFiHardware, 10, 10, 512, 1024> nh;  // Reduced subs/pubs for safety on ESP32
 
 volatile long encoder_fl = 0, encoder_fr = 0, encoder_rl = 0, encoder_rr = 0;
 long prev_encoder_fl = 0, prev_encoder_fr = 0, prev_encoder_rl = 0, prev_encoder_rr = 0;
 int motor_speeds[4] = {0, 0, 0, 0};
 
 float target_vel_fl = 0, target_vel_fr = 0, target_vel_rl = 0, target_vel_rr = 0;
 float current_vel_fl = 0, current_vel_fr = 0, current_vel_rl = 0, current_vel_rr = 0;
 float error_sum_fl = 0, error_sum_fr = 0, error_sum_rl = 0, error_sum_rr = 0;
 float prev_error_fl = 0, prev_error_fr = 0, prev_error_rl = 0, prev_error_rr = 0;
 
 float odom_x = 0, odom_y = 0, odom_theta = 0;
 MPU9250 mpu;
 
 float kp = 14.0, ki = 2.0, kd = 0.4;
 const int MIN_PWM = 85;
 const float WHEEL_RADIUS = 0.0325;
 const float TRACK_WIDTH = 0.26;
 const int TICKS_PER_REV = 4900;
 
 unsigned long last_control_time = 0, last_cmd_time = 0, last_imu_time = 0, last_ros_connect = 0;
 
 nav_msgs::Odometry odom_msg;
 sensor_msgs::Imu imu_msg;
 ros::Publisher odom_pub("wheel_odom", &odom_msg);
 ros::Publisher imu_pub("imu/data", &imu_msg);
 
 // ==================== ENCODER ISRs ====================
 void IRAM_ATTR isrFL() { digitalRead(ENC_FL_A) == digitalRead(ENC_FL_B) ? encoder_fl-- : encoder_fl++; }
 void IRAM_ATTR isrFR() { digitalRead(ENC_FR_A) == digitalRead(ENC_FR_B) ? encoder_fr-- : encoder_fr++; }
 void IRAM_ATTR isrRL() { digitalRead(ENC_RL_A) == digitalRead(ENC_RL_B) ? encoder_rl-- : encoder_rl++; }
 void IRAM_ATTR isrRR() { digitalRead(ENC_RR_A) == digitalRead(ENC_RR_B) ? encoder_rr-- : encoder_rr++; }
 
 // ==================== MOTOR CONTROL ====================
 void software_pwm_loop() {
   static unsigned long last_pwm_time = 0;
   unsigned long now = micros();
 
   if (now - last_pwm_time >= 1000) {
     last_pwm_time = now;
     static uint8_t pwm_counter = 0;
     pwm_counter++;
 
     digitalWrite(MOTOR_FL_PWM, (motor_speeds[0] > pwm_counter) ? HIGH : LOW);
     digitalWrite(MOTOR_FR_PWM, (motor_speeds[1] > pwm_counter) ? HIGH : LOW);
     digitalWrite(MOTOR_RL_PWM, (motor_speeds[2] > pwm_counter) ? HIGH : LOW);
     digitalWrite(MOTOR_RR_PWM, (motor_speeds[3] > pwm_counter) ? HIGH : LOW);
   }
 }
 
 void setMotorSpeed(int motor, int pwm_value) {
   pwm_value = constrain(pwm_value, -255, 255);
 
   int in1, in2;
   switch (motor) {
     case 0: in1 = MOTOR_FL_IN1; in2 = MOTOR_FL_IN2; break;
     case 1: in1 = MOTOR_FR_IN1; in2 = MOTOR_FR_IN2; break;
     case 2: in1 = MOTOR_RL_IN1; in2 = MOTOR_RL_IN2; break;
     case 3: in1 = MOTOR_RR_IN1; in2 = MOTOR_RR_IN2; break;
     default: return;
   }
 
   if (pwm_value > 5) {
     digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
   } else if (pwm_value < -5) {
     digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
   } else {
     digitalWrite(in1, LOW); digitalWrite(in2, LOW);
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
   error_sum = constrain(error_sum, -100, 100);
 
   float derivative = (dt > 0) ? (error - prev_error) / dt : 0;
   prev_error = error;
 
   int pwr = (int)(kp * error + ki * error_sum + kd * derivative);
 
   if (pwr > 0) pwr += MIN_PWM;
   else if (pwr < 0) pwr -= MIN_PWM;
 
   return constrain(pwr, -255, 255);
 }
 
 void updateOdometry(float dt) {
   if (dt <= 0) return;
 
   float v_left  = (current_vel_fl + current_vel_rl) / 2.0;
   float v_right = (current_vel_fr + current_vel_rr) / 2.0;
   float v_linear = (v_left + v_right) / 2.0;
   float v_angular = (v_right - v_left) / TRACK_WIDTH;
 
   odom_theta += v_angular * dt;
   odom_x += v_linear * cos(odom_theta) * dt;
   odom_y += v_linear * sin(odom_theta) * dt;
 
   odom_msg.header.stamp = nh.now();
   odom_msg.pose.pose.position.x = odom_x;
   odom_msg.pose.pose.position.y = odom_y;
   odom_msg.pose.pose.orientation.z = sin(odom_theta * 0.5);
   odom_msg.pose.pose.orientation.w = cos(odom_theta * 0.5);
   odom_msg.twist.twist.linear.x = v_linear;
   odom_msg.twist.twist.angular.z = v_angular;
 
   odom_pub.publish(&odom_msg);
 }
 
 void publishIMU() {
   if (mpu.update()) {
     imu_msg.header.stamp = nh.now();
     imu_msg.angular_velocity.x = mpu.getGyroX();
     imu_msg.angular_velocity.y = mpu.getGyroY();
     imu_msg.angular_velocity.z = mpu.getGyroZ();
     imu_msg.linear_acceleration.x = mpu.getAccX() * 9.81;
     imu_msg.linear_acceleration.y = mpu.getAccY() * 9.81;
     imu_msg.linear_acceleration.z = mpu.getAccZ() * 9.81;
     imu_pub.publish(&imu_msg);
   }
 }
 
 void controlLoop() {
   float dt = 0.05;
   float ticks_to_m = (2.0 * PI * WHEEL_RADIUS) / TICKS_PER_REV;
 
   noInterrupts();
   long enc_fl = encoder_fl, enc_fr = encoder_fr;
   long enc_rl = encoder_rl, enc_rr = encoder_rr;
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
   delay(2000);
   Serial.println("\n\n=== ESP32 4WD Robot Controller ===");
 
   int outPins[] = {MOTOR_FL_IN1, MOTOR_FL_IN2, MOTOR_FL_PWM,
                    MOTOR_FR_IN1, MOTOR_FR_IN2, MOTOR_FR_PWM,
                    MOTOR_RL_IN1, MOTOR_RL_IN2, MOTOR_RL_PWM,
                    MOTOR_RR_IN1, MOTOR_RR_IN2, MOTOR_RR_PWM};
   for (int i = 0; i < 12; i++) {
     pinMode(outPins[i], OUTPUT);
     digitalWrite(outPins[i], LOW);
   }
 
   pinMode(ENC_FL_A, INPUT); pinMode(ENC_FL_B, INPUT);
   pinMode(ENC_FR_A, INPUT); pinMode(ENC_FR_B, INPUT);
   pinMode(ENC_RL_A, INPUT_PULLUP); pinMode(ENC_RL_B, INPUT_PULLUP);
   pinMode(ENC_RR_A, INPUT_PULLUP); pinMode(ENC_RR_B, INPUT_PULLUP);
 
   Serial.print("Connecting to WiFi: ");
   Serial.println(ssid);
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) {
     delay(500);
     Serial.print(".");
   }
   Serial.println("\nWiFi connected! IP: ");
   Serial.println(WiFi.localIP());
 
   Wire.begin(IMU_SDA, IMU_SCL);
   mpu.setup(0x68);
   delay(100);
   mpu.calibrateAccelGyro();
 
   attachInterrupt(digitalPinToInterrupt(ENC_FL_A), isrFL, CHANGE);
   attachInterrupt(digitalPinToInterrupt(ENC_FR_A), isrFR, CHANGE);
   attachInterrupt(digitalPinToInterrupt(ENC_RL_A), isrRL, CHANGE);
   attachInterrupt(digitalPinToInterrupt(ENC_RR_A), isrRR, CHANGE);
 
   // ROS setup
   *nh.getHardware() = ros_wifi_hw;
   nh.initNode();
   nh.advertise(odom_pub);
   nh.advertise(imu_pub);
   nh.subscribe(cmd_vel_sub);
 
   odom_msg.header.frame_id = "odom";
   odom_msg.child_frame_id = "base_footprint";
 
   Serial.println("Setup complete. Waiting for ROS connection...");
 }
 
 // ==================== LOOP ====================
 void loop() {
   unsigned long now = millis();
 
   if (!ros_wifi_hw.connected()) {
     if (now - last_ros_connect > 2000) {
       ros_wifi_hw.connect();
       last_ros_connect = now;
     }
   } else {
     nh.spinOnce();
 
     if (now - last_control_time >= 50) {
       controlLoop();
       last_control_time = now;
     }
 
     if (now - last_cmd_time > 800) {
       target_vel_fl = target_vel_fr = target_vel_rl = target_vel_rr = 0;
     }
 
     if (now - last_imu_time >= 100) {
       publishIMU();
       last_imu_time = now;
     }
   }
 
   software_pwm_loop();
   delay(1);
 }