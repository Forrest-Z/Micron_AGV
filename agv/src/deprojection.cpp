#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include <obstacle_detector/Obstacles.h>

#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>

#define BASELINK_TO_CAMERA 2.2 // in meters

using std::cout;
using std::endl;
using std::vector;

// Functions (not methods)
constexpr double pi() { return M_PI; }
double deg2rad(double angle) { return angle * pi() / 180; }
double rad2deg(double angle) { return angle / pi() * 180; }

class Deprojection
{
public:
  // Constructor
  Deprojection();

  // Destructor
  virtual ~Deprojection(){};

  // Public Variables

  // Public Methods

private:
  // Private Variables
  double deprojection_frequency_;

  const double FOV_ = 78; // in degrees

  double car_x = 10.0;
  double car_y = 10.0;
  double car_yaw = deg2rad(45);

  const vector<double> img_size = {1280, 720}; // width and height

  vector<vector<double>> camera_pose; //Position of camera in world map [x, y]
  vector<vector<double>> obstacle_in_image; // position of obstacle in image [class_id, width, height, depth, centre x, centre y]
  vector<vector<double>> obstacle_angle_and_radius; // angle of center point of obstacle, radius of the obstacle
  vector<vector<double>> obstacle_wrtGlobal; //3D obstacle data wrt World Frame [class_id, width/2, height/2, obstacle_x, obstacle_y]

  // ROS Subscribers
  ros::Subscriber camera_obstacle_sub;
  ros::Subscriber odom_sub;

  // ROS Publishers
  ros::Publisher deprojected_obstacle_pub;

  // Variables and Functions for subscribe to odom topic
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  //timer
  ros::Timer timer;

  // Private Methods / Functions
  void getImageObstacleAngleAndRadius();
  void deprojectImage();
  void publishDeprojectionResult();

  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
  void cameraImageCallBack(const std_msgs::Float64MultiArray::ConstPtr &obstacle_msg);

  void mainTimerCallback(const ros::TimerEvent& timer_event);
};

//My Own Constructor
Deprojection::Deprojection() : tf_listener(tf_buffer)
{
  ros::NodeHandle private_nh("~");
  // topics
	std::string odom_topic_;
  std::string deprojection_topic_;
  std::string camera_obstacle_topic_;

  // ROS_ASSERT (to assign constant values in launch file. No need to compile when changes are made to these values)
  ROS_ASSERT(private_nh.getParam("deprojection_frequency", deprojection_frequency_)); //do we need a freq?? I think so
  
  ROS_ASSERT(private_nh.getParam("odom_topic", odom_topic_));
  ROS_ASSERT(private_nh.getParam("camera_obstacle_topic", camera_obstacle_topic_));
  ROS_ASSERT(private_nh.getParam("deprojection_topic", deprojection_topic_));

  // Subscribe & Advertise
  odom_sub = private_nh.subscribe(odom_topic_, 1, &Deprojection::odomCallback, this);
  camera_obstacle_sub = private_nh.subscribe(camera_obstacle_topic_, 1, &Deprojection::cameraImageCallBack, this);

  // publish to another topic
  deprojected_obstacle_pub = private_nh.advertise<obstacle_detector::Obstacles>(deprojection_topic_, 1);

  // Initialize the timer
  timer = private_nh.createTimer(ros::Duration(1.0 / deprojection_frequency_), &Deprojection::mainTimerCallback, this);
}

void Deprojection::mainTimerCallback(const ros::TimerEvent& timer_event)
{
  ROS_DEBUG("timer start");

  // TODO: (07/11) Check if all required data are in position

  if (obstacle_in_image.size() == 0)  // check if got input from camera
  {
    ROS_WARN("Empty");
    return;
  }

  // Call the methods
  getImageObstacleAngleAndRadius();

  deprojectImage();

  publishDeprojectionResult();

  // Clear all global arrays
  camera_pose.clear();
  obstacle_in_image.clear();
  obstacle_angle_and_radius.clear();
  obstacle_wrtGlobal.clear();
}

////////////////////////////////////////CALLBACKS///////////////////////////////////

//Retrieve co-ordinates of buggy wrt Global Frame
void Deprojection::odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
  geometry_msgs::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf_buffer.lookupTransform("map", "odom", ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }

  geometry_msgs::PoseStamped pose_before_transform, pose_after_transform;
  pose_before_transform.header.frame_id = odom_msg->header.frame_id;
  pose_before_transform.header.stamp = odom_msg->header.stamp;
  pose_before_transform.pose = odom_msg->pose.pose;
  tf2::doTransform(pose_before_transform, pose_after_transform, transform_stamped);

  tf::Quaternion q(pose_after_transform.pose.orientation.x, pose_after_transform.pose.orientation.y,
                   pose_after_transform.pose.orientation.z, pose_after_transform.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, car_yaw);

  // Current XY of robot (map frame)
  car_x = pose_after_transform.pose.position.x;
  car_y = pose_after_transform.pose.position.y;
}

//Retrieve camera(ZED) detected obstacle data
void Deprojection::cameraImageCallBack(const std_msgs::Float64MultiArray::ConstPtr &obstacle_msg)
{
  ROS_DEBUG("camera callback start");
  for (int i=0; i < obstacle_msg->data.size()/6; i++)
  {
    double class_id = obstacle_msg->data[0];
    double width = obstacle_msg->data[1];
    double height = obstacle_msg->data[2];
    double depth = obstacle_msg->data[3];
    double center_x = obstacle_msg->data[4];
    double center_y = obstacle_msg->data[5];

    obstacle_in_image.push_back({class_id, width, height, depth, center_x, center_y});
    ROS_DEBUG("camera callback survive");
    //remember to find array index of depth first! 
    // ZHIJIE WILL HANDLE THIS
  }

  return;
} 

/////////////////////////////////////////METHODS////////////////////////////////////

void Deprojection::getImageObstacleAngleAndRadius()
{
  ROS_DEBUG("getImageObstacleAngleAndRadius method");

  for (int i = 0; i < obstacle_in_image.size(); i++)
  {
    double alpha = deg2rad(FOV_ / 2);

    // index 4 of obstacle_in_image is center_x
    double img_theta_center = atan(tan(alpha) * ((img_size[0] / 2) - obstacle_in_image[i][4]) / (img_size[0] / 2));
    ROS_DEBUG("img_theta_center computed");
    cout << "theta_center = " << img_theta_center << endl;

    // index 4 of obstacle_in_image is center_x, index 1 is width, index 3 is depth
    double img_theta_left = atan(tan(alpha) * ((img_size[0] / 2) - obstacle_in_image[i][4] + obstacle_in_image[i][1]/ 2) / (img_size[0] / 2));
    double delta_y = obstacle_in_image[i][3] * sin(img_theta_center);
    double delta_x = obstacle_in_image[i][3] * cos(img_theta_center);
    double left = delta_x * tan(img_theta_left);
    double radius = left - delta_y;
    
    vector <double> temp = {img_theta_center,radius}; // first element is angle, second element is radius
    obstacle_angle_and_radius.push_back(temp);
    
    ROS_DEBUG("obstacle radius survive");
  }
  ROS_DEBUG("getImageObstacleAngleAndRadius method finish");
  return;
}

void Deprojection::deprojectImage()
{
  for (int i = 0; i<obstacle_in_image.size(); i++)
  {
    // Find camera pose wrt global frame

    // int COL = 10; int ROW = 10; 
    // camera_pose.resize(COL, vector<double>(ROW));

    double x = car_x + BASELINK_TO_CAMERA * cos(car_yaw);  //camera_x
    double y = car_y + BASELINK_TO_CAMERA * sin(car_yaw);  //camera_y
    camera_pose.push_back({x,y});

    // Find delta x & y of obstacle from camera. x is perpendicular to camera, y is parallel to camera
    double delta_x = obstacle_in_image[i][3] * cos(obstacle_angle_and_radius[i][0]);
    double delta_y = obstacle_in_image[i][3] * sin(obstacle_angle_and_radius[i][0]);

    // Find pose of obstacles wrt global frame
    double obstacle_x = camera_pose[i][0] + delta_x * cos(car_yaw) - delta_y * sin(car_yaw);
    double obstacle_y = camera_pose[i][1] + delta_x * sin(car_yaw) + delta_y * cos(car_yaw);

    // Save obstacle x & y data into 2D vector [x, y]
    // obstacle_wrtGlobal.reserve(100);
    // obstacle_wrtGlobal[i].push_back(obstacle_x); // first element is x
    // obstacle_wrtGlobal[i].push_back(obstacle_y); // second element is y
    obstacle_wrtGlobal.push_back({obstacle_x, obstacle_y});
  }
  ROS_DEBUG("deprojectImage method finish");
  return;
}

void Deprojection::publishDeprojectionResult()
{
  ROS_DEBUG("publishDeprojectionResult method start");
  obstacle_detector::Obstacles deprojection_msg;

  deprojection_msg.header.frame_id = "map";

  for (int i=0; i < obstacle_wrtGlobal.size(); i++)
  {
    obstacle_detector::CircleObstacle circle;

    circle.true_radius = obstacle_in_image[i][0];
    circle.radius = obstacle_angle_and_radius[i][1];
    circle.center.x = obstacle_wrtGlobal[i][0];
    circle.center.y = obstacle_wrtGlobal[i][1];

    deprojection_msg.circles.emplace_back(circle);
    
    // deprojection_msg.circles.reserve(5); //push_back(temp);
    // deprojection_msg.circles[i].true_radius = obstacle_in_image[i][0];        //obstacle class id
    // deprojection_msg.circles[i].radius = obstacle_angle_and_radius[i][1];     //obstacle radius
    // deprojection_msg.circles[i].center.x = obstacle_wrtGlobal[i][0];          //obstacle_x
    // deprojection_msg.circles[i].center.y = obstacle_wrtGlobal[i][1];          //obstacle_y
  }
  
  deprojected_obstacle_pub.publish(deprojection_msg);
  ROS_DEBUG("publishDeprojectionResult method finish");
  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "deprojection_node");
  // Construct a deprojection object
  Deprojection deprojecton_obj; // = SensorFusion();
  ros::spin();                    //spin the ros node.
  return 0;
}