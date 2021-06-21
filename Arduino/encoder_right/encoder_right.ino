/*
 * Date: 12 April 2019`
 */
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>
#include <Encoder.h>

ros::NodeHandle nh;

//Create objects
Encoder knob(19, 18);

// encoder publishers

std_msgs::Int64 encoder_count_msg;
ros::Publisher encoder_count_pub("raw_encoder_count_right", &encoder_count_msg);

long encoderCount = 0;


// encoder publish function
void Publish(){
    encoder_count_msg.data = encoderCount;
    encoder_count_pub.publish ( &encoder_count_msg );
}

void messageCb(const std_msgs::Bool& msg) {
  encoderCount = knob.read();
  Publish();
}

ros::Subscriber<std_msgs::Bool> sub("encoder_timer", &messageCb);

void setup() {
  nh.initNode();
  nh.advertise(encoder_count_pub);
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
}
 
