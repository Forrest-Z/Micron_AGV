/*
 * This node:
 * 1) Receives a String representing the file name of the path
 * 2) Reads the path from the file
 * 3) Publishes the path as a latched topic
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <fstream>

class PathPublisher
{
public:
  PathPublisher();

private:
  ros::NodeHandle nh;
  ros::Subscriber path_filename_sub;
  ros::Publisher path_pub;

  std::string paths_directory;
  std::string current_path_filename;
  bool create_reversed_path_flag;

  void pathFilenameCallback(const std_msgs::String::ConstPtr& path_filename_msg);
  void publishPathFromFile();
  void createReversedPathFile(nav_msgs::Path);
};

PathPublisher::PathPublisher()
{
  ros::NodeHandle private_nh("~");

  std::string path_filename_topic;
  std::string path_topic;

  ROS_ASSERT(private_nh.getParam("path_filename_topic", path_filename_topic));
  ROS_ASSERT(private_nh.getParam("path_topic", path_topic));
  ROS_ASSERT(private_nh.getParam("paths_directory", paths_directory));
  ROS_ASSERT(private_nh.getParam("create_reversed_path_flag", create_reversed_path_flag));

  path_filename_sub = nh.subscribe(path_filename_topic, 1, &PathPublisher::pathFilenameCallback, this);
  path_pub = nh.advertise<nav_msgs::Path>(path_topic, 1, true);
}

void PathPublisher::pathFilenameCallback(const std_msgs::String::ConstPtr& path_filename_msg)
{
  current_path_filename = path_filename_msg->data;
  publishPathFromFile();
}

void PathPublisher::publishPathFromFile()
{
  std::ifstream in_file;
  std::string file_path = paths_directory + current_path_filename;
  in_file.open(file_path);
  if (!in_file)
  {
    ROS_ERROR("Path Publisher: Could not open path file!!!");
    return;
  }
  else
  {
    ROS_INFO("Path Publisher: Reading a path file.");
  }

  nav_msgs::Path current_path;
  current_path.header.frame_id = "map";
  double x, y;
  while ((in_file >> x) && (in_file >> y))
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(0);
    pose.pose.orientation = pose_quat;
    current_path.poses.emplace_back(pose);
  }

  // double x, y, z;
  // while ((in_file >> x) && (in_file >> y) && (in_file >> z))
  // {
  //   geometry_msgs::PoseStamped pose;
  //   pose.header.frame_id = "map";
  //   pose.pose.position.x = x;
  //   pose.pose.position.y = y;
  //   pose.pose.position.z = z;
  //   geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(0);
  //   pose.pose.orientation = pose_quat;
  //   current_path.poses.emplace_back(pose);
  // }

  path_pub.publish(current_path);

  in_file.close();

  if (create_reversed_path_flag)
  {
    createReversedPathFile(current_path);
  }
}

void PathPublisher::createReversedPathFile(nav_msgs::Path path)
{
  // std::string reversed_path_filename = "/home/agv/catkin_ws/src/f10agv/agv/paths/reversed_" + current_path_filename;
  std::string reversed_path_filename = paths_directory + "reversed_" + current_path_filename;
  std::ofstream myfile;
  myfile.open(reversed_path_filename);
  if (myfile.is_open())
  {
    for (int i = (path.poses.size() - 1); i >= 0; i--)
    {
      myfile << path.poses[i].pose.position.x << " " << path.poses[i].pose.position.y << std::endl;
    }
  }
  myfile.close();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_publisher_node");
  PathPublisher path_publisher_obj;
  ros::spin();
  return 0;
}
