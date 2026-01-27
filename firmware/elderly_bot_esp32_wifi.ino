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
 #include <tf/tf.h>
 #include <tf/transform_broadcaster.h>
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

 // PWM channels
 const int CH_FL = 0;
 const int CH_FR = 1;
 const int CH_RL = 2;
 const int CH_RR = 3;

 // Robot parameters
 const float TRACK_WIDTH = 0.26; // meters
 const float WHEEL_RADIUS = 0.0325; // meters
 const float TICKS_PER_REV = 3960.0;
 const float DISTANCE_PER_TICK = 2 * M_PI * WHEEL_RADIUS / TICKS_PER_REV;
 const int PWM_FREQ = 5000;
 const int PWM_RES = 8;
 const int PWM_MAX = 255;
 const float MAX_SPEED = 0.25; // m/s

 // Encoder counts and debounce
 volatile long counts[4] = {0, 0, 0, 0}; // FL, FR, RL, RR
 unsigned long last_interrupt_time[4] = {0, 0, 0, 0};

 // Odometry
 float odom_x = 0.0;
 float odom_y = 0.0;
 float odom_theta = 0.0;

 // Target velocities from cmd_vel
 float target_linear_x = 0.0;
 float target_angular_z = 0.0;

 // ROS
 ros::NodeHandle nh;
 nav_msgs::Odometry odom_msg;
 ros::Publisher odom_pub("odom", &odom_msg);
 void cmd_vel_cb(const geometry_msgs::Twist& twist_msg) {
   target_linear_x = twist_msg.linear.x;
   target_angular_z = twist_msg.angular.z;
 }
 ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmd_vel_cb);

 unsigned long last_odom_time = 0;
 const int ODOM_PUBLISH_INTERVAL = 50; // 20Hz

 TaskHandle_t motorTaskHandle;
 WiFiClient client;
 RosWiFiHardware ros_wifi_hw(&client, jetson_ip, SERVER_PORT);
 unsigned long last_ros_connect = 0;
 Preferences preferences;

 // ISR functions
 void isrFL() {
   unsigned long now = micros();
   if (now - last_interrupt_time[0] > 100) { // debounce 100us
     int A = digitalRead(FL_ENC_A);
     int B = digitalRead(FL_ENC_B);
     if (A == B) counts[0]--;
     else counts[0]++;
     last_interrupt_time[0] = now;
   }
 }

 void isrFR() {
   unsigned long now = micros();
   if (now - last_interrupt_time[1] > 100) {
     int A = digitalRead(FR_ENC_A);
     int B = digitalRead(FR_ENC_B);
     if (A == B) counts[1]--; // May need inversion for direction
     else counts[1]++;
     last_interrupt_time[1] = now;
   }
 }

 void isrRL() {
   unsigned long now = micros();
   if (now - last_interrupt_time[2] > 100) {
     int A = digitalRead(RL_ENC_A);
     int B = digitalRead(RL_ENC_B);
     if (A == B) counts[2]--;
     else counts[2]++;
     last_interrupt_time[2] = now;
   }
 }

 void isrRR() {
   unsigned long now = micros();
   if (now - last_interrupt_time[3] > 100) {
     int A = digitalRead(RR_ENC_A);
     int B = digitalRead(RR_ENC_B);
     if (A == B) counts[3]--;
     else counts[3]++;
     last_interrupt_time[3] = now;
   }
 }

 // Update odometry
 void updateOdometry(float dt) {
   long delta_counts_left = (counts[0] + counts[2]) / 2;
   long delta_counts_right = (counts[1] + counts[3]) / 2;
   // Reset counts
   for (int i = 0; i < 4; i++) counts[i] = 0;

   float delta_left = delta_counts_left * DISTANCE_PER_TICK;
   float delta_right = delta_counts_right * DISTANCE_PER_TICK;

   float delta_distance = (delta_left + delta_right) / 2.0;
   float delta_theta = (delta_right - delta_left) / TRACK_WIDTH;

   float dx = delta_distance * cos(odom_theta + delta_theta / 2.0);
   float dy = delta_distance * sin(odom_theta + delta_theta / 2.0);

   odom_x += dx;
   odom_y += dy;
   odom_theta += delta_theta;

   if (odom_theta > M_PI) odom_theta -= 2 * M_PI;
   if (odom_theta < -M_PI) odom_theta += 2 * M_PI;

   float vx = delta_distance / dt;
   float vy = 0.0;
   float vth = delta_theta / dt;

   odom_msg.header.stamp = nh.now();
   odom_msg.pose.pose.position.x = odom_x;
   odom_msg.pose.pose.position.y = odom_y;
   odom_msg.pose.pose.position.z = 0.0;
   geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(odom_theta);
   odom_msg.pose.pose.orientation = odom_quat;
   odom_msg.twist.twist.linear.x = vx;
   odom_msg.twist.twist.linear.y = vy;
   odom_msg.twist.twist.angular.z = vth;
 }

 // Publish odometry
 void publishOdometry() {
   // Add covariances (higher for theta and vyaw due to skid-steer slip)
   double pose_cov[36] = {0.01, 0, 0, 0, 0, 0,  // x variance
                          0, 0.01, 0, 0, 0, 0,  // y
                          0, 0, 0.01, 0, 0, 0,  // z
                          0, 0, 0, 0.001, 0, 0, // roll
                          0, 0, 0, 0, 0.001, 0, // pitch
                          0, 0, 0, 0, 0, 0.1};  // yaw (higher due to slip)
   memcpy(odom_msg.pose.covariance, pose_cov, sizeof(pose_cov));

   double twist_cov[36] = {0.01, 0, 0, 0, 0, 0,  // vx variance
                           0, 0.01, 0, 0, 0, 0,  // vy
                           0, 0, 0.01, 0, 0, 0,  // vz
                           0, 0, 0, 0.001, 0, 0, // vroll
                           0, 0, 0, 0, 0.001, 0, // vpitch
                           0, 0, 0, 0, 0, 0.1};  // vyaw (higher due to slip)
   memcpy(odom_msg.twist.covariance, twist_cov, sizeof(twist_cov));

   odom_pub.publish(&odom_msg);
 }

 // Motor control task (runs on Core 1)
 void motorControlTask(void * parameter) {
   for (;;) {
     float v_left = target_linear_x - target_angular_z * (TRACK_WIDTH / 2.0);
     float v_right = target_linear_x + target_angular_z * (TRACK_WIDTH / 2.0);

     int pwm_left = (int)(v_left * PWM_MAX / MAX_SPEED);
     int pwm_right = (int)(v_right * PWM_MAX / MAX_SPEED);

     pwm_left = constrain(pwm_left, -PWM_MAX, PWM_MAX);
     pwm_right = constrain(pwm_right, -PWM_MAX, PWM_MAX);

     // Left motors (FL, RL)
     if (pwm_left > 0) {
       digitalWrite(FL_IN1, HIGH);
       digitalWrite(FL_IN2, LOW);
       ledcWrite(CH_FL, pwm_left);
       digitalWrite(RL_IN1, HIGH);
       digitalWrite(RL_IN2, LOW);
       ledcWrite(CH_RL, pwm_left);
     } else if (pwm_left < 0) {
       digitalWrite(FL_IN1, LOW);
       digitalWrite(FL_IN2, HIGH);
       ledcWrite(CH_FL, -pwm_left);
       digitalWrite(RL_IN1, LOW);
       digitalWrite(RL_IN2, HIGH);
       ledcWrite(CH_RL, -pwm_left);
     } else {
       ledcWrite(CH_FL, 0);
       ledcWrite(CH_RL, 0);
     }

     // Right motors (FR, RR)
     if (pwm_right > 0) {
       digitalWrite(FR_IN1, HIGH);
       digitalWrite(FR_IN2, LOW);
       ledcWrite(CH_FR, pwm_right);
       digitalWrite(RR_IN1, HIGH);
       digitalWrite(RR_IN2, LOW);
       ledcWrite(CH_RR, pwm_right);
     } else if (pwm_right < 0) {
       digitalWrite(FR_IN1, LOW);
       digitalWrite(FR_IN2, HIGH);
       ledcWrite(CH_FR, -pwm_right);
       digitalWrite(RR_IN1, LOW);
       digitalWrite(RR_IN2, HIGH);
       ledcWrite(CH_RR, -pwm_right);
     } else {
       ledcWrite(CH_FR, 0);
       ledcWrite(CH_RR, 0);
     }

     vTaskDelay(10 / portTICK_PERIOD_MS); // 100Hz loop
   }
 }

 void setup() {
   // Load saved Jetson IP
   preferences.begin("ros_ip", false);
   String saved_ip = preferences.getString("jetson_ip", "");
   preferences.end();

   jetson_ip = IPAddress(0,0,0,0);
   if (jetson_ip.fromString(saved_ip)) {
     ros_ip_valid = true;
   } else {
     // Launch WiFiManager to configure
     WiFiManager wifiManager;
     WiFiManagerParameter custom_jetson_ip("jetson_ip", "Jetson IP", jetson_ip_buffer, 16);
     wifiManager.addParameter(&custom_jetson_ip);
     wifiManager.autoConnect("ElderlyBotESP32");
     strcpy(jetson_ip_buffer, custom_jetson_ip.getValue());
     preferences.begin("ros_ip", false);
     preferences.putString("jetson_ip", jetson_ip_buffer);
     preferences.end();
     jetson_ip.fromString(jetson_ip_buffer);
     ros_ip_valid = true;
   }

   // Motor pins
   pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
   pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT);
   pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT);
   pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT);

   // PWM setup
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