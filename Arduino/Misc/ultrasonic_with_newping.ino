#include <ros.h>
#include <std_msgs/Float64.h>
#include <NewPing.h>

#define TRIGGER_PIN 11
#define ECHO_PIN 12
#define MAX_DISTANCE 400

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 

ros::NodeHandle nh;
std_msgs::Float64 ultrasonic_detection_distance_msg;
ros::Publisher ultrasonic_pub("/ultrasonic_output_front", &ultrasonic_detection_distance_msg);

void setup() {
  nh.initNode();
  nh.advertise(ultrasonic_pub);
}

void loop() {
  delay(100);
  unsigned int distance = sonar.ping_cm();
  ultrasonic_detection_distance_msg.data = distance / 100.0;
  ultrasonic_pub.publish( &ultrasonic_detection_distance_msg );
  nh.spinOnce();
}
