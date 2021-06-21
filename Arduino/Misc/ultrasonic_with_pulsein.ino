#include <ros.h>
#include <std_msgs/Float64.h>

#define TRIGGER_PIN 11
#define ECHO_PIN 12

#define DURATION_TO_DISTANCE_CONST 5820.0

long duration;
double distance;  // in metres

ros::NodeHandle nh;
std_msgs::Float64 ultrasonic_detection_distance_msg;
ros::Publisher ultrasonic_pub("/ultrasonic_output_front", &ultrasonic_detection_distance_msg);

void readUltrasonicSensor()
{
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  pinMode(ECHO_PIN, INPUT);
  duration = pulseIn(ECHO_PIN, HIGH, 100000);

  distance = duration / DURATION_TO_DISTANCE_CONST;
  ultrasonic_detection_distance_msg.data = distance;
}

void setup()
{
  nh.initNode();
  nh.advertise(ultrasonic_pub);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop()
{
  readUltrasonicSensor();
  ultrasonic_pub.publish(&ultrasonic_detection_distance_msg);
  nh.spinOnce();
  delay(100);
}
