// Sample template for writing ROS classes in OOP

#include <ros/ros.h>
#include <ros/console.h>

class ClassName
{
public:
  ClassName();

private:
  ros::NodeHandle nh;
  ros::Subscriber example_sub;
  ros::Publisher example_pub;

  // Declare other variables here

  // Declare callbacks
  void exampleCallback(...);

  // Declare other functions
};

// constructor
ClassName::ClassName()
{
  ros::NodeHandle private_nh("~");

  //  Declare the topics
  std::string example_topic;

  // Get params from launch file
  ROS_ASSERT(private_nh.getParam("example_topic", example_topic));

  // Subscribe and/or Advertise
  example_sub = nh.subscribe(example_topic, 1, &ClassName::exampleCallback, this);
  example_pub = nh.advertise<...>(example_topic, 1);

  // Other initializations
}

void ClassName::exampleCallback(...)
{
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "class_name_node");
  ClassName class_name_obj;
  ros::spin();
  return 0;
}
