/*
 * Unified ESP32 Firmware - Ground Truth Motor Behavior + ROS WiFi Communication
 * Motor control logic: EXACTLY from motor_and_encoder_HW_test.ino
 * ROS/WiFi communication: From elderly_bot_esp32_wifi.ino ground truth
 */
 #include <WiFi.h>
 #include <WiFiClient.h>
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
 // ==================== WiFi CONFIGURATION ====================
 const char* ssid = "ShellBack";
 const char* password = "hhmo@1974";
 IPAddress server(192, 168, 1, 16);
 const uint16_t serverPort = 11411;
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
 #define ODOM_PUBLISH_INTERVAL 100 // 10Hz odometry (100ms)
 #define SAFETY_TIMEOUT 800 // Stop motors if no cmd_vel for 800ms
 #define ROS_SERIAL_BUFFER_SIZE 1024
 // ==================== GLOBALS ====================
 WiFiClient client;
 RosWiFiHardware ros_wifi_hw(&client, server, serverPort);
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
 const float WHEEL_RADIUS = 0.0194;  // Final calibration: 19.4mm radius (38.8mm diameter) - corrected for 1.046m test
 const float TRACK_WIDTH = 0.26;
 const int TICKS_PER_REV = 990;
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
       // Calculate left/right speeds as specified: left = linear.x - angular.z, right = linear.x + angular.z
       float left_speed = local_cmd_vel.linear_x - local_cmd_vel.angular_z;
       float right_speed = local_cmd_vel.linear_x + local_cmd_vel.angular_z;
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
   // Convert encoder ticks to velocity (m/s)
   float ticks_to_m = (2.0 * M_PI * WHEEL_RADIUS) / TICKS_PER_REV;
   float vel_fl = enc_fl * ticks_to_m / dt;
   float vel_fr = enc_fr * ticks_to_m / dt;
   float vel_rl = enc_rl * ticks_to_m / dt;
   float vel_rr = enc_rr * ticks_to_m / dt;
   // Differential drive odometry
   float v_left = (vel_fl + vel_rl) / 2.0;
   float v_right = (vel_fr + vel_rr) / 2.0;
   current_linear = (v_left + v_right) / 2.0;
   current_angular = (v_right - v_left) / TRACK_WIDTH;
   // Update pose
   odom_theta += current_angular * dt;
   odom_x += current_linear * cos(odom_theta) * dt;
   odom_y += current_linear * sin(odom_theta) * dt;
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
 // ==================== SETUP (Ground Truth Hardware Initialization) ====================
 void setup() {
   Serial.begin(115200);
   // Safe Boot Delay (EXACT from ground truth)
   pinMode(FL_IN1, INPUT); pinMode(RL_IN2, INPUT); delay(500);
   // Create mutex for cmd_vel shared data
   cmd_vel_mutex = xSemaphoreCreateMutex();
   // Connect to WiFi
   WiFi.begin(ssid, password);
   Serial.print("Connecting to WiFi");
   while(WiFi.status() != WL_CONNECTED) {
     delay(100);
     Serial.print(".");
   }
   Serial.println("\nWiFi connected!");
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
   
   // ROS setup (Core 0)
   *nh.getHardware() = ros_wifi_hw;
   nh.initNode();
   nh.subscribe(cmd_vel_sub);
   nh.advertise(odom_pub);
   odom_msg.header.frame_id = "odom";
   odom_msg.child_frame_id = "base_footprint";
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
   Serial.println("Unified ESP32 Firmware Ready - Dual Core!");
 }
 // ==================== LOOP (Core 0 - ROS/WiFi/Odometry Only) ====================
 void loop() {
   unsigned long now = millis();
   // Connection management
   if (!ros_wifi_hw.connected()) {
     if (now - last_ros_connect > 2000) {
        ros_wifi_hw.init();
        last_ros_connect = now;
     }
   }
   nh.spinOnce();
   // Publish odometry (10Hz = every 100ms) - secondary priority
   if (now - last_odom_time >= ODOM_PUBLISH_INTERVAL) {
     updateOdometry(ODOM_PUBLISH_INTERVAL / 1000.0);
     publishOdometry();
     last_odom_time = now;
   }
   // Small delay to prevent overwhelming rosserial
   delay(1);
 }