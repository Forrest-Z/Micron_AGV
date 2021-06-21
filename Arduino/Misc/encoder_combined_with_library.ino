// Updated 14 June 2019

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>
#include <Encoder.h>

ros::NodeHandle nh;

Encoder left_knob(18,19);
Encoder right_knob(2,3);

long left_encoder_count;
std_msgs::Int64 left_encoder_count_msg;
ros::Publisher left_encoder_count_pub("raw_encoder_count_left", &left_encoder_count_msg);

long right_encoder_count;
std_msgs::Int64 right_encoder_count_msg;
ros::Publisher right_encoder_count_pub("raw_encoder_count_right", &right_encoder_count_msg);

void Publish() {
  left_encoder_count_msg.data = left_encoder_count;
  left_encoder_count_pub.publish(&left_encoder_count_msg);
  
  right_encoder_count_msg.data = right_encoder_count;
  right_encoder_count_pub.publish(&right_encoder_count_msg);
}

void timerCb() {
  left_encoder_count = left_knob.read();
  right_encoder_count = right_knob.read();
  Publish();
}

ros::Subscriber<std_msgs::Bool> timer_sub("encoder_timer", &timerCb);

void setup() {
  nh.initNode();
  nh.advertise(left_encoder_count_pub);
  nh.advertise(right_encoder_count_pub);
  nh.subscribe(timer_sub);
}

void loop() {
  nh.spinOnce();
}
