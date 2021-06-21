/*
 * This node is used to convert the cloud from merged laser scan
 * into a format suitable for the object detection package.
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

class PCL2ToPCLConverter
{
public:
  PCL2ToPCLConverter();

private:
  ros::Subscriber PCL2_sub;
  ros::Publisher PCL_pub;
  ros::NodeHandle nh;

  void cloudInCallback(const sensor_msgs::PointCloud2::ConstPtr& pcl2_msg);
};

PCL2ToPCLConverter::PCL2ToPCLConverter()
{
  ros::NodeHandle private_nh("~");

  std::string cloud_in_topic;
  std::string cloud_out_topic;

  ROS_ASSERT(private_nh.getParam("cloud_in_topic", cloud_in_topic));
  ROS_ASSERT(private_nh.getParam("cloud_out_topic", cloud_out_topic));

  PCL2_sub = nh.subscribe(cloud_in_topic, 1, &PCL2ToPCLConverter::cloudInCallback, this);
  PCL_pub = nh.advertise<sensor_msgs::PointCloud>(cloud_out_topic, 1);
}

void PCL2ToPCLConverter::cloudInCallback(const sensor_msgs::PointCloud2::ConstPtr& pcl2_msg)
{
  sensor_msgs::PointCloud pcl;
  sensor_msgs::convertPointCloud2ToPointCloud(*pcl2_msg, pcl);
  PCL_pub.publish(pcl);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl2_to_pcl_converter");
  PCL2ToPCLConverter pcl2_to_pcl_converter_obj;
  ros::spin();
  return 0;
}
