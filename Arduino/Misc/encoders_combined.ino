/*
 * Arduino Mega3 code, for Encoders, using 3 Arduinos
 * Implementation: Just uses interrupts to count number of pulses,
 * does not account for reversing vehicle
 * Date: 29 March 2019
 */

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>

ros::NodeHandle nh;

// encoder publishers
std_msgs::Int64 encoder_count_msg_left;
ros::Publisher encoder_count_pub_left("raw_encoder_count_left", &encoder_count_msg_left);

std_msgs::Int64 encoder_count_msg_right;
ros::Publisher encoder_count_pub_right("raw_encoder_count_right", &encoder_count_msg_right);

//variables for two encoders
long countLeft = 0;
long countRight = 0;


// encoder publish function
void Publish(){
    encoder_count_msg_left.data = countLeft;
    encoder_count_msg_right.data = countRight;
    encoder_count_pub_left.publish( &encoder_count_msg_left );
    encoder_count_pub_right.publish ( &encoder_count_msg_right);
}

void messageCb(const std_msgs::Bool& msg) {
  Publish();
}

void ISR1() {
  countLeft++;
}

void ISR2() {
  countRight++;
}

ros::Subscriber<std_msgs::Bool> sub("encoder_timer", &messageCb);

void setup() {
  nh.initNode();
  nh.advertise(encoder_count_pub_left);
  nh.advertise(encoder_count_pub_right);
  nh.subscribe(sub);
  attachInterrupt(digitalPinToInterrupt(2), ISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(18), ISR2, RISING);
}

void loop() {
  nh.spinOnce();
}
