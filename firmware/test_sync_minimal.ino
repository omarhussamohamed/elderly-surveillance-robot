/*
 * MINIMAL ROS TEST - For Sync Diagnosis
 * 
 * This is the absolute minimum code to test rosserial sync.
 * If this doesn't work, the issue is definitely library/version related.
 * 
 * Expected behavior:
 * 1. Upload this to ESP32
 * 2. Wait 5 seconds after boot
 * 3. Run: rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
 * 4. Should see: "Setup subscriber on test [std_msgs/String]"
 * 
 * If this works but main firmware doesn't, issue is in main firmware code.
 * If this doesn't work, issue is library/version mismatch.
 */

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

void messageCallback(const std_msgs::String& msg) {
  // Do nothing - just for testing
}

ros::Subscriber<std_msgs::String> sub("test", &messageCallback);
std_msgs::String str_msg;
ros::Publisher pub("test_pub", &str_msg);

void setup() {
  // CRITICAL: Minimal setup, NO Serial output during sync
  Serial.begin(115200);
  delay(500);  // Wait for Serial to stabilize
  
  // Initialize ROS - NO output until connected
  nh.getHardware()->setBaud(115200);
  delay(200);
  
  nh.initNode();
  
  // Wait for sync (up to 10 seconds)
  unsigned long timeout = millis() + 10000;
  while(!nh.connected() && millis() < timeout) {
    nh.spinOnce();
    delay(10);
  }
  
  // Only configure after sync
  nh.subscribe(sub);
  nh.advertise(pub);
  
  // NOW safe to print
  if(nh.connected()) {
    Serial.println("ROS SYNC SUCCESSFUL!");
  } else {
    Serial.println("ROS SYNC FAILED!");
  }
}

void loop() {
  nh.spinOnce();
  delay(10);
}

