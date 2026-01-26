/*
 * Unified ESP32 Firmware - Ground Truth Motor Behavior + ROS WiFi Communication
 * Motor control logic: EXACTLY from motor_and_encoder_HW_test.ino
 * ROS/WiFi communication: From elderly_bot_esp32_wifi.ino ground truth
 * Wi-Fi Configuration: WiFiManager with persistent Jetson IP via Preferences
 */
 #include <WiFi.h>
 #include <WiFiClient.h>
 #include <WiFiManager.h>
 #include <Preferences.h>
 #include <ros.h>
 #include <geometry_msgs/Twist.h>
 #include <nav_msgs/Odometry.h>
 #include <math.h>
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
   bool connected() { return (client && client->connected()); }
 };
 // ==================== WiFi AND ROS IP CONFIGURATION ====================
 // Wi-Fi credentials and ROS IP are configured via WiFiManager captive portal
 // and persisted using Preferences.h
 const uint16_t SERVER_PORT = 11411;
 IPAddress jetson_ip;  // Will be loaded from Preferences
 bool ros_ip_valid = false;  // Track if we have a valid Jetson IP
 // Global WiFiManager parameter (must persist beyond setup)
 char jetson_ip_buffer[16] = "";
 // ==================== GROUND TRUTH MOTOR SETTINGS (FROM motor_and_encoder_HW_test.ino) ====================
 // Pin definitions - EXACT from ground truth
 const int FL_PWM = 13; const int FL_IN1 = 12; const int FL_IN2 = 14;
 const int FL_ENC_A = 34; const int FL_ENC_B = 35;
 const int FR_PWM = 27; const int FR_IN1 = 26; const int FR_IN2 = 25;
 const int FR_ENC_A = 36; const int FR_ENC_B = 39;
 const int RL_PWM = 21; const int RL_IN1 = 32; const int RL_IN2 = 15;
 const int RL_ENC_A = 18; const int RL_ENC_B = 19;
 const int RR_PWM = 22; const int RR_IN1 = 16; const int RR_IN2 = 17;
 const int RR_ENC_A = 23; const int RR_ENC_B = 5;
 // Settings - EXACT from ground truth
 const int MIN_MOVE_PWM = 152;
 const int PWM_FREQ = 5000;
 const int PWM_RES = 8;
 const int CH_FL = 0; const int CH_FR = 1; const int CH_RL = 4; const int CH_RR = 5;
 // ==================== TIMING SETTINGS ====================
 #define CONTROL_LOOP_INTERVAL 20 // 50Hz control loop (20ms)
 #define ODOM_PUBLISH_INTERVAL 50 // 20Hz odometry (50ms) - LAG FIX: Reduce sensor staleness without full rate matching
 #define SAFETY_TIMEOUT 800 // Stop motors if no cmd_vel for 800ms
 #define ROS_SERIAL_BUFFER_SIZE 1024
 // ==================== GLOBALS ====================
 WiFiClient client;
 // Note: ros_wifi_hw initialized with placeholder IP, will be configured in setup()
 RosWiFiHardware ros_wifi_hw(&client, IPAddress(0, 0, 0, 0), SERVER_PORT);
 ros::NodeHandle_<RosWiFiHardware, 25, 25, ROS_SERIAL_BUFFER_SIZE, ROS_SERIAL_BUFFER_SIZE> nh;
 // Ground truth encoder counters - EXACT from motor_and_encoder_HW_test.ino
 volatile long counts[4] = {0, 0, 0, 0}; // FL, FR, RL, RR
// Encoder debounce timing (microseconds)
volatile unsigned long last_interrupt_time[4] = {0, 0, 0, 0}; // FL, FR, RL, RR
const unsigned long DEBOUNCE_MICROS = 100; // 100 microseconds - proven optimal, eliminates static creep
 // Odometry state
 float odom_x = 0, odom_y = 0, odom_theta = 0;
 // Current velocities for odom twist
 float current_linear = 0.0;
 float current_angular = 0.0;
 // Physical constants
 const float WHEEL_RADIUS = 0.0325;  // 32.5mm radius (65mm diameter) - STRICT KINEMATIC SPECIFICATION
 const float TRACK_WIDTH = 0.26;
 const int TICKS_PER_REV = 3960;  // JGB37-520: 11 PPR × 90 gear ratio × 4 edges = 3960
 // Timing
 unsigned long last_control_time = 0, last_cmd_time = 0, last_odom_time = 0, last_ros_connect = 0;
 // Shared structure for cmd_vel (rosserial writes, motor task reads atomically)
 struct CmdVelData {
   float linear_x;
   float angular_z;
   unsigned long timestamp;
   bool updated;
 } cmd_vel_shared;
 // Motor control state (accessed only by motor task)
 bool motor_direction_forward = true;
 int motor_power_left = 0;
 int motor_power_right = 0;
 // Mutex for cmd_vel shared data
 SemaphoreHandle_t cmd_vel_mutex;
 // Motor control task handle
 TaskHandle_t motorTaskHandle;
 // ROS messages
 nav_msgs::Odometry odom_msg;
 ros::Publisher odom_pub("wheel_odom", &odom_msg);
 // ==================== GROUND TRUTH ENCODER ISRs (EXACT from motor_and_encoder_HW_test.ino) ====================
 // These ensure Forward = Positive across all 4 wheels
// Software debounce added to prevent noise-induced false counts

void IRAM_ATTR isrFL() {
  unsigned long current_time = micros();
  if (current_time - last_interrupt_time[0] > DEBOUNCE_MICROS) {
    digitalRead(FL_ENC_A) == digitalRead(FL_ENC_B) ? counts[0]-- : counts[0]++;
    last_interrupt_time[0] = current_time;
  }
}

void IRAM_ATTR isrFR() {
  unsigned long current_time = micros();
  if (current_time - last_interrupt_time[1] > DEBOUNCE_MICROS) {
    digitalRead(FR_ENC_A) == digitalRead(FR_ENC_B) ? counts[1]-- : counts[1]++;
    last_interrupt_time[1] = current_time;
  }
}

void IRAM_ATTR isrRL() {
  unsigned long current_time = micros();
  if (current_time - last_interrupt_time[2] > DEBOUNCE_MICROS) {
    digitalRead(RL_ENC_A) == digitalRead(RL_ENC_B) ? counts[2]-- : counts[2]++;
    last_interrupt_time[2] = current_time;
  }
}

void IRAM_ATTR isrRR() {
  unsigned long current_time = micros();
  if (current_time - last_interrupt_time[3] > DEBOUNCE_MICROS) {
    digitalRead(RR_ENC_A) == digitalRead(RR_ENC_B) ? counts[3]-- : counts[3]++;
    last_interrupt_time[3] = current_time;
  }
}
 // ==================== GROUND TRUTH MOTOR CONTROL (EXACT from motor_and_encoder_HW_test.ino) ====================
 void setLeftDirection(bool forward) {
   if (forward) {
     digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
     digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, HIGH);
   } else {
     digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
     digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, LOW);
   }
 }
 void setRightDirection(bool forward) {
   if (forward) {
     digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
     digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH);
   } else {
     digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
     digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
   }
 }
 void driveLeft(int pwr) {
   int effective_pwr = (pwr == 0) ? 0 : map(pwr, 1, 255, MIN_MOVE_PWM, 255);
   ledcWrite(CH_FL, effective_pwr);
   ledcWrite(CH_RL, effective_pwr);
 }
 void driveRight(int pwr) {
   int effective_pwr = (pwr == 0) ? 0 : map(pwr, 1, 255, MIN_MOVE_PWM, 255);
   ledcWrite(CH_FR, effective_pwr);
   ledcWrite(CH_RR, effective_pwr);
 }
 void stopLeft() {
   driveLeft(0);
   digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW);
   digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, LOW);
 }
 void stopRight() {
   driveRight(0);
   digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, LOW);
   digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, LOW);
 }
 void stopMotors() {
   stopLeft();
   stopRight();
 }
 // ==================== MOTOR CONTROL TASK (Core 1) ====================
 void motorControlTask(void *pvParameters) {
   const TickType_t xFrequency = pdMS_TO_TICKS(CONTROL_LOOP_INTERVAL);
   while (true) {
     // Read cmd_vel from shared structure atomically
     CmdVelData local_cmd_vel;
     bool has_new_cmd = false;
     if (xSemaphoreTake(cmd_vel_mutex, portMAX_DELAY) == pdTRUE) {
       if (cmd_vel_shared.updated) {
         local_cmd_vel = cmd_vel_shared;
         cmd_vel_shared.updated = false; // Mark as processed
         has_new_cmd = true;
       }
       xSemaphoreGive(cmd_vel_mutex);
     }
     if (has_new_cmd) {
       // Calculate left/right speeds using correct differential drive kinematics
       // For differential drive: v_left = v - omega * (track_width/2), v_right = v + omega * (track_width/2)
       float left_speed = local_cmd_vel.linear_x - (local_cmd_vel.angular_z * TRACK_WIDTH / 2.0);
       float right_speed = local_cmd_vel.linear_x + (local_cmd_vel.angular_z * TRACK_WIDTH / 2.0);
       // Convert to PWM [0-255], using abs for power
       int left_pwr = constrain((int)(fabs(left_speed) * 255.0), 0, 255);
       int right_pwr = constrain((int)(fabs(right_speed) * 255.0), 0, 255);
       bool left_forward = (left_speed >= 0);
       bool right_forward = (right_speed >= 0);
       // Drive left side
       if (left_pwr > 0) {
         setLeftDirection(left_forward);
         driveLeft(left_pwr);
       } else {
         stopLeft();
       }
       // Drive right side
       if (right_pwr > 0) {
         setRightDirection(right_forward);
         driveRight(right_pwr);
       } else {
         stopRight();
       }
     }
     // Check safety timeout
     if (millis() - last_cmd_time > SAFETY_TIMEOUT) {
       stopMotors();
     }
     vTaskDelay(xFrequency);
   }
 }
 // ==================== ODOMETRY FUNCTIONS ====================
 void updateOdometry(float dt) {
   if (dt <= 0) return;
   // Read encoder counts atomically
   noInterrupts();
   long enc_fl = counts[0], enc_fr = counts[1];
   long enc_rl = counts[2], enc_rr = counts[3];
   interrupts();
   // Convert encoder ticks to velocity (m/s) - STRICT FLOAT MATH
   float ticks_to_m = (2.0 * M_PI * WHEEL_RADIUS) / (float)TICKS_PER_REV;
   float vel_fl = enc_fl * ticks_to_m / dt;
   float vel_fr = enc_fr * ticks_to_m / dt;
   float vel_rl = enc_rl * ticks_to_m / dt;
   float vel_rr = enc_rr * ticks_to_m / dt;
   // Differential drive odometry
   float v_left = (vel_fl + vel_rl) / 2.0;
   float v_right = (vel_fr + vel_rr) / 2.0;
   
   // Calculate instantaneous velocities
   float linear_vel = (v_left + v_right) / 2.0;
   float angular_vel = (v_right - v_left) / TRACK_WIDTH;
   
   // Apply dead-zone threshold to eliminate stationary encoder noise
   const float VEL_THRESHOLD = 0.001;  // 1mm/s - below this is considered stationary
   if (fabs(linear_vel) < VEL_THRESHOLD) linear_vel = 0.0;
   if (fabs(angular_vel) < 0.001) angular_vel = 0.0;  // 0.001 rad/s ~0.06°/s
   
   // Store for publishing
   current_linear = linear_vel;
   current_angular = angular_vel;
   
   // CRITICAL: Correct pose integration using MIDPOINT theta (Runge-Kutta 2nd order)
   // This prevents systematic circular drift from one-step-ahead errors
   float theta_old = odom_theta;
   odom_theta += angular_vel * dt;
   
   // Use midpoint theta for position integration (average of old and new)
   float theta_mid = theta_old + (angular_vel * dt / 2.0);
   odom_x += linear_vel * cos(theta_mid) * dt;
   odom_y += linear_vel * sin(theta_mid) * dt;
   // Normalize theta
   while (odom_theta > M_PI) odom_theta -= 2 * M_PI;
   while (odom_theta < -M_PI) odom_theta += 2 * M_PI;
   // Reset encoder counts for next iteration
   noInterrupts();
   counts[0] = counts[1] = counts[2] = counts[3] = 0;
   interrupts();
 }
 void publishOdometry() {
   odom_msg.header.stamp = nh.now();
   odom_msg.pose.pose.position.x = odom_x;
   odom_msg.pose.pose.position.y = odom_y;
   odom_msg.pose.pose.orientation.z = sin(odom_theta * 0.5);
   odom_msg.pose.pose.orientation.w = cos(odom_theta * 0.5);
   
   // Set realistic covariance for wheel odometry (EKF needs this)
   // ROTATION FIX: Dynamically scale yaw covariance based on linear velocity
   // During pure rotation (low vx), wheel slip is extreme → high yaw covariance (distrust encoders)
   // During linear motion, wheel odom is more reliable → lower yaw covariance
   
   float abs_linear = fabs(current_linear);
   float abs_angular = fabs(current_angular);
   
   // Diagonal covariance: [x, y, z, rot_x, rot_y, rot_z] for pose
   odom_msg.pose.covariance[0] = 0.001;   // x position variance (1mm std)
   odom_msg.pose.covariance[7] = 0.001;   // y position variance
   
   // Dynamic yaw covariance: High during rotation, low during linear motion
   // If vx < 0.05 m/s AND rotating → pure rotation mode (high slip)
   if (abs_linear < 0.05 && abs_angular > 0.1) {
     odom_msg.pose.covariance[35] = 1.0;  // 1.0 rad² variance (VERY HIGH - extreme slip during pure rotation)
   } else if (abs_linear < 0.1) {
     odom_msg.pose.covariance[35] = 0.5;  // 0.5 rad² (high slip during slow motion)
   } else {
     odom_msg.pose.covariance[35] = 0.05; // 0.05 rad² (normal - forward motion more reliable)
   }
   
   // Twist covariance: [vx, vy, vz, vroll, vpitch, vyaw]
   odom_msg.twist.covariance[0] = 0.002;   // vx variance
   odom_msg.twist.covariance[7] = 0.002;   // vy variance
   
   // Dynamic vyaw covariance: VERY high during pure rotation
   if (abs_linear < 0.05 && abs_angular > 0.1) {
     odom_msg.twist.covariance[35] = 2.0;  // Extremely unreliable from wheels during pure rotation
   } else {
     odom_msg.twist.covariance[35] = 0.5;  // High but not extreme during normal motion
   }
   
   odom_msg.twist.twist.linear.x = current_linear;
   odom_msg.twist.twist.angular.z = current_angular;
   odom_pub.publish(&odom_msg);
 }
 // ==================== CMD_VEL CALLBACK ====================
 void cmdVelCallback(const geometry_msgs::Twist& cmd_vel) {
   last_cmd_time = millis();
   // Update shared structure atomically
   if (xSemaphoreTake(cmd_vel_mutex, portMAX_DELAY) == pdTRUE) {
     cmd_vel_shared.linear_x = cmd_vel.linear.x;
     cmd_vel_shared.angular_z = cmd_vel.angular.z;
     cmd_vel_shared.timestamp = millis();
     cmd_vel_shared.updated = true;
     xSemaphoreGive(cmd_vel_mutex);
   }
 }
 ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmdVelCallback);
 // ==================== WIFI AND ROS IP CONFIGURATION FUNCTIONS ====================
 void saveJetsonIP(String ip_string) {
   Preferences prefs;
   prefs.begin("ros_cfg", false);
   prefs.putString("jetson_ip", ip_string);
   prefs.end();
 }
 String loadJetsonIP() {
   Preferences prefs;
   prefs.begin("ros_cfg", true);  // Read-only mode
   String ip_string = prefs.getString("jetson_ip", "");
   prefs.end();
   return ip_string;
 }
 void configureWiFiAndROS() {
   // Force station mode and disable sleep for reliability
   WiFi.mode(WIFI_STA);
   WiFi.setSleep(false);
   
   // Preload stored IP into buffer for portal
   String stored_ip = loadJetsonIP();
   if (stored_ip.length() > 0) {
     stored_ip.toCharArray(jetson_ip_buffer, 16);
   }
   
   WiFiManager wm;
   
   // Custom parameter for Jetson IP (uses global buffer)
   WiFiManagerParameter custom_jetson_ip("jetson_ip", "Jetson ROS Master IP", jetson_ip_buffer, 16);
   wm.addParameter(&custom_jetson_ip);
   
   // Set custom AP name
   wm.setConfigPortalTimeout(180);  // 3 minute timeout
   
   // Launch captive portal
   if (!wm.autoConnect("ElderlyBot_Config_AP")) {
     delay(3000);
     ESP.restart();
   }
   
   // Wi-Fi connected
   
   // Get custom parameter value
   String jetson_ip_string = custom_jetson_ip.getValue();
   
   // Validate IP format first
   IPAddress temp_ip;
   if (jetson_ip_string.length() > 0 && temp_ip.fromString(jetson_ip_string)) {
     // Valid IP entered - save and use it
     jetson_ip = temp_ip;
     saveJetsonIP(jetson_ip_string);
     ros_ip_valid = true;
   } else if (stored_ip.length() > 0 && jetson_ip.fromString(stored_ip)) {
     // No new IP, use stored IP if valid
     ros_ip_valid = true;
   } else {
     // No valid IP available
     ros_ip_valid = false;
   }
 }
 // ==================== SETUP (Ground Truth Hardware Initialization) ====================
 void setup() {
   
   // Safe Boot Delay (EXACT from ground truth)
   pinMode(FL_IN1, INPUT); pinMode(RL_IN2, INPUT); delay(500);
   
   // Create mutex for cmd_vel shared data
   cmd_vel_mutex = xSemaphoreCreateMutex();
   
   // Configure WiFi and load Jetson IP via WiFiManager
   configureWiFiAndROS();
   
   // Only proceed with ROS setup if we have a valid IP
   if (ros_ip_valid) {
     // Set ROS connection with validated IP
     ros_wifi_hw.setConnection(jetson_ip, SERVER_PORT);
     client.setNoDelay(true);  // Disable Nagle algorithm for low latency
   }
   
   // Initialize all Outputs (EXACT from ground truth)
   int pins[] = {FL_IN1, FL_IN2, FR_IN1, FR_IN2, RL_IN1, RL_IN2, RR_IN1, RR_IN2, FL_PWM, FR_PWM, RL_PWM, RR_PWM};
   for(int p : pins) {
     pinMode(p, OUTPUT);
     digitalWrite(p, LOW);
   }
   
   // HW PWM Setup (EXACT from ground truth)
   ledcSetup(CH_FL, PWM_FREQ, PWM_RES); ledcSetup(CH_FR, PWM_FREQ, PWM_RES);
   ledcSetup(CH_RL, PWM_FREQ, PWM_RES); ledcSetup(CH_RR, PWM_FREQ, PWM_RES);
   ledcAttachPin(FL_PWM, CH_FL); ledcAttachPin(FR_PWM, CH_FR);
   ledcAttachPin(RL_PWM, CH_RL); ledcAttachPin(RR_PWM, CH_RR);
   
   // Encoders (EXACT from ground truth - Internal Pullups for RL/RR, External 1k for FR)
   pinMode(FL_ENC_A, INPUT); pinMode(FL_ENC_B, INPUT);
   pinMode(FR_ENC_A, INPUT); pinMode(FR_ENC_B, INPUT);
   pinMode(RL_ENC_A, INPUT_PULLUP); pinMode(RL_ENC_B, INPUT_PULLUP);
   pinMode(RR_ENC_A, INPUT_PULLUP); pinMode(RR_ENC_B, INPUT_PULLUP);
   
   // Attach interrupts (EXACT from ground truth)
   attachInterrupt(digitalPinToInterrupt(FL_ENC_A), isrFL, CHANGE);
   attachInterrupt(digitalPinToInterrupt(FR_ENC_A), isrFR, CHANGE);
   attachInterrupt(digitalPinToInterrupt(RL_ENC_A), isrRL, CHANGE);
   attachInterrupt(digitalPinToInterrupt(RR_ENC_A), isrRR, CHANGE);
   
   // Reset encoder counts and odometry to zero on startup
   for (int i = 0; i < 4; i++) {
     counts[i] = 0;
     last_interrupt_time[i] = 0;
   }
   odom_x = 0.0;
   odom_y = 0.0;
   odom_theta = 0.0;
   
   // ROS setup (Core 0) - only if valid IP
   if (ros_ip_valid) {
     *nh.getHardware() = ros_wifi_hw;
     ros_wifi_hw.init();  // Initiate initial TCP connection to ROS Master
     nh.initNode();
     nh.subscribe(cmd_vel_sub);
     nh.advertise(odom_pub);
     odom_msg.header.frame_id = "odom";
     odom_msg.child_frame_id = "base_footprint";
   }
   
   // Create motor control task on Core 1
   xTaskCreatePinnedToCore(
     motorControlTask, // Task function
     "MotorControl", // Name
     2048, // Stack size
     NULL, // Parameters
     1, // Priority
     &motorTaskHandle, // Handle
     1 // Core 1
   );
 }
 // ==================== LOOP (Core 0 - ROS/WiFi/Odometry Only) ====================
 void loop() {
   unsigned long now = millis();
   
   // Only run ROS operations if we have a valid IP
   if (ros_ip_valid) {
     // Connection management
     if (!ros_wifi_hw.connected()) {
       if (now - last_ros_connect > 1000) {
          client.stop();  // Clean disconnect before reconnect
          ros_wifi_hw.init();
          last_ros_connect = now;
       }
     }
     nh.spinOnce();
     // Publish odometry (20Hz)
     if (now - last_odom_time >= ODOM_PUBLISH_INTERVAL) {
       updateOdometry(ODOM_PUBLISH_INTERVAL / 1000.0);
       publishOdometry();
       last_odom_time = now;
     }
   }
   // Small delay to prevent overwhelming rosserial
   delay(1);
 }