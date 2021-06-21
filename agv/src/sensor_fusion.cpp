#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include <obstacle_detector/Obstacles.h>
#include <obstacle_detector/CircleObstacle.h>

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

#define BASELINK_TO_CAMERA 2.2 // in meters

using std::cout;
using std::endl;
using std::vector;

// Functions (not methods)
constexpr double pi() { return M_PI; }
double deg2rad(double angle) { return angle * pi() / 180; }
double rad2deg(double angle) { return angle / pi() * 180; }

class SensorFusion
{
public:
  // Constructor
  SensorFusion();

  // Destructor
  virtual ~SensorFusion(){};

  // Public Variables

  // Public Methods

private:
  // Private Variables
  double sensor_fusion_frequency_;

  const double FOV_ = 85; // in degrees

  double car_x;
  double car_y;
  double car_yaw;

  const vector<double> img_size = {1280, 720}; // width and height

  vector<vector<double>> obstacle_in_scan; // position of obstacle from 3D LiDAR scan wrt World Frame[x, y, radius, vx, vy]
  vector<double> camera_pose; // position of camera wrt World Frame [x, y]
  vector<vector<double>> obstacle_in_image; // position of obstacle in image [class_id, width, height, depth, center_x, center_y]
  vector<vector<double>> obstacle_angle_and_radius; // [angle of center point of obstacle, radius of the obstacle]
  vector<vector<double>> obstacle_wrtGlobal; //3D obstacle data wrt World Frame [obstacle_x, obstacle_y]

  // ROS Subscribers
  ros::Subscriber lidar_obstacle_sub;
  ros::Subscriber camera_obstacle_sub;
  ros::Subscriber odom_sub;

  // ROS Publishers
  ros::Publisher fused_obstacle_pub;
  ros::Publisher deprojected_obstacle_pub;

  // Variables and Functions for subscribe to odom topic
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  //timer
  ros::Timer timer;

  // Private Methods / Functions
  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
  void obstacleCallback(const obstacle_detector::Obstacles::ConstPtr &obstacle_msg);
  void cameraImageCallBack(const std_msgs::Float64MultiArray::ConstPtr &obstacle_msg);  
  
  void getImageObstacleAngleAndRadius();
  void deprojectImage();
  void compareLidarWithCamera();

  void publishSensorFusionResult();
  void publishDeprojectionResult();

  void mainTimerCallback(const ros::TimerEvent& timer_event);
};

//My Own Constructor
SensorFusion::SensorFusion() : tf_listener(tf_buffer)
{
  ros::NodeHandle private_nh("~");
  // topics
	std::string odom_topic_;
  std::string sensor_fusion_topic_;
  std::string camera_obstacle_topic_;
  std::string obstacle_topic_;
  std::string deprojection_topic_;

  // ROS_ASSERT (to assign constant values in launch file. No need to compile when changes are made to these values)
  ROS_ASSERT(private_nh.getParam("sensor_fusion_frequency", sensor_fusion_frequency_)); //do we need a freq?? I think so
  
  ROS_ASSERT(private_nh.getParam("odom_topic", odom_topic_));
  ROS_ASSERT(private_nh.getParam("obstacle_topic", obstacle_topic_));    
  ROS_ASSERT(private_nh.getParam("camera_obstacle_topic", camera_obstacle_topic_));
  ROS_ASSERT(private_nh.getParam("sensor_fusion_topic", sensor_fusion_topic_));
  ROS_ASSERT(private_nh.getParam("deprojection_topic", deprojection_topic_));

  // Subscribe & Advertise
  odom_sub = private_nh.subscribe(odom_topic_, 1, &SensorFusion::odomCallback, this);
  camera_obstacle_sub = private_nh.subscribe(camera_obstacle_topic_, 1, &SensorFusion::cameraImageCallBack, this);
  lidar_obstacle_sub = private_nh.subscribe(obstacle_topic_,1,&SensorFusion::obstacleCallback, this);

  // publish to another topic
  fused_obstacle_pub = private_nh.advertise<obstacle_detector::Obstacles>(sensor_fusion_topic_, 1);
  deprojected_obstacle_pub = private_nh.advertise<obstacle_detector::Obstacles>(deprojection_topic_, 1);

  // Initialize the timer
  timer = private_nh.createTimer(ros::Duration(1.0 / sensor_fusion_frequency_), &SensorFusion::mainTimerCallback, this);
}

void SensorFusion::mainTimerCallback(const ros::TimerEvent& timer_event)
{
  ROS_DEBUG("timer start");

  // Check if all required data are in position

  if (obstacle_in_image.size() == 0)  // check if got input from camera
  {
    ROS_WARN("Camera empty");
    return;
  }

  if (obstacle_in_scan.size() == 0)  // check if got input from Lidar
  {
    ROS_WARN("Lidar empty");
    return;
  }

  // Call the methods
  getImageObstacleAngleAndRadius();
  deprojectImage();
  compareLidarWithCamera();

  publishSensorFusionResult();
  publishDeprojectionResult();

  // Clear all global arrays
  camera_pose.clear();
  obstacle_in_scan.clear();
  obstacle_in_image.clear();
  obstacle_angle_and_radius.clear();
  obstacle_wrtGlobal.clear();
}

////////////////////////////////////////CALLBACKS///////////////////////////////////

//Retrieve co-ordinates of buggy wrt Global Frame
void SensorFusion::odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
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

//Retrieve 3D LiDAR detected obstacle data
void SensorFusion::obstacleCallback(const obstacle_detector::Obstacles::ConstPtr &obstacle_msg)
{
  ROS_DEBUG("lidar callback start");
  for (int i = 0; i < obstacle_msg->circles.size(); i++)
  {
    double x = obstacle_msg->circles[i].center.x;
    double y = obstacle_msg->circles[i].center.y;
    double tr = obstacle_msg->circles[i].true_radius;
    double vx = obstacle_msg->circles[i].velocity.x;
    double vy = obstacle_msg->circles[i].velocity.y;
    double r = obstacle_msg->circles[i].radius;

    obstacle_in_scan.push_back({x, y, tr, vx, vy, r});
  }
  return;
}

//Retrieve camera(ZED) detected obstacle data
void SensorFusion::cameraImageCallBack(const std_msgs::Float64MultiArray::ConstPtr &obstacle_msg)
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

void SensorFusion::getImageObstacleAngleAndRadius()
{
  ROS_DEBUG("getImageObstacleAngleAndRadius method");

  for (int i = 0; i < obstacle_in_image.size(); i++)
  {
    double alpha = deg2rad(FOV_ / 2);

    // index 4 of obstacle_in_image is center_x
    double img_theta_center = atan(tan(alpha) * ((img_size[0] / 2) - obstacle_in_image[i][4]) / (img_size[0] / 2));
    cout << "theta_center = " << img_theta_center << endl;

    // index 4 of obstacle_in_image is center_x, index 1 is width, index 3 is depth
    double img_theta_left = atan(tan(alpha) * ((img_size[0] / 2) - obstacle_in_image[i][4] + obstacle_in_image[i][1]/ 2) / (img_size[0] / 2));
    double delta_y = obstacle_in_image[i][3] * sin(img_theta_center);
    double delta_x = obstacle_in_image[i][3] * cos(img_theta_center);
    double left = delta_x * tan(img_theta_left);
    double radius = left - delta_y;
    
    vector <double> temp = {img_theta_center,radius}; // first element is angle, second element is radius
    obstacle_angle_and_radius.push_back(temp);
  }
  ROS_DEBUG("getImageObstacleAngleAndRadius method finish");
  return;
}

void SensorFusion::deprojectImage()
{
    double x = car_x + BASELINK_TO_CAMERA * cos(car_yaw);  //camera_x
    double y = car_y + BASELINK_TO_CAMERA * sin(car_yaw);  //camera_y
    camera_pose.push_back(x);
    camera_pose.push_back(y);

  for (int i = 0; i<obstacle_in_image.size(); i++)
  {
    // Find camera pose wrt global frame

    // int COL = 10; int ROW = 10; 
    // camera_pose.resize(COL, vector<double>(ROW));

    // Find delta x & y of obstacle from camera. x is perpendicular to camera, y is parallel to camera
    double delta_x = obstacle_in_image[i][3] * cos(obstacle_angle_and_radius[i][0]);
    double delta_y = obstacle_in_image[i][3] * sin(obstacle_angle_and_radius[i][0]);

    // Find pose of obstacles wrt global frame
    double obstacle_x = camera_pose[0] + delta_x * cos(car_yaw) - delta_y * sin(car_yaw);
    double obstacle_y = camera_pose[1] + delta_x * sin(car_yaw) + delta_y * cos(car_yaw);

    // Save obstacle x & y data into 2D vector [x, y]
    obstacle_wrtGlobal.push_back({obstacle_x, obstacle_y});
    cout<<"x from camera = "<<obstacle_x<<"y from camera = "<<obstacle_y<<endl;
  }
  ROS_DEBUG("deprojectImage method finish");
  return;
}

void SensorFusion::compareLidarWithCamera()
{
  for (int i=0; i < obstacle_wrtGlobal.size(); i++)
  {
    cout<<"camera obstacle radius = "<<obstacle_angle_and_radius[i][1]<<endl;
    for (int j=0; j < obstacle_in_scan.size(); j++)
    {
      double d = sqrt(pow(obstacle_wrtGlobal[i][0]-obstacle_in_scan[j][0], 2)+pow(obstacle_wrtGlobal[i][1]-obstacle_in_scan[j][1], 2)); // distance between circles from center to center
      cout<<"d = "<<d<<endl;
      // check if the center of lidar circle lies inside the camera circle or not
      if (d < obstacle_angle_and_radius[i][1])                  // Yes, take the class id
      {
        obstacle_in_scan[j].push_back(obstacle_in_image[i][0]);
        ROS_DEBUG("matched");
      }
      else                                                      // No, set class id = 0
      {
        obstacle_in_scan[j].push_back(0);
        ROS_DEBUG("not matched");
      }
    }
    ROS_DEBUG("compareLidarWithCamera method finish");
  }
  ROS_DEBUG("compared lidar with camera");
  return;
}

void SensorFusion::publishSensorFusionResult()
{
  ROS_DEBUG("publishSensorFusionResult method start");
  obstacle_detector::Obstacles sensor_fusion_msg;

  sensor_fusion_msg.header.frame_id = "map";  // display on map frame

  for (int i=0; i < obstacle_in_scan.size(); i++)
  {
    obstacle_detector::CircleObstacle circle;

    circle.center.x = obstacle_in_scan[i][0];
    circle.center.y = obstacle_in_scan[i][1];    
    circle.true_radius = obstacle_in_scan[i][2];  // true radius is true radius
    circle.velocity.x = obstacle_in_scan[i][3];
    circle.velocity.y = obstacle_in_scan[i][4];
    circle.radius = obstacle_in_scan[i][5];
    circle.class_id = obstacle_in_scan[i][6];       // radius is class id

    sensor_fusion_msg.circles.emplace_back(circle);
  }
  
  fused_obstacle_pub.publish(sensor_fusion_msg);
  ROS_DEBUG("publishSensorFusionResult method finish");
  return;
}

void SensorFusion::publishDeprojectionResult()
{
  ROS_DEBUG("publishDeprojectionResult method start");
  obstacle_detector::Obstacles deprojection_msg;

  deprojection_msg.header.frame_id = "map";

  for (int i=0; i < obstacle_wrtGlobal.size(); i++)
  {
    obstacle_detector::CircleObstacle circle;

    circle.true_radius = obstacle_angle_and_radius[i][1];   //obstacle true radius
    circle.radius = obstacle_angle_and_radius[i][1]*1.1;   //obstacle true radius
    circle.class_id = obstacle_in_image[i][0];              //obstacle class id
    circle.center.x = obstacle_wrtGlobal[i][0];             //obstacle_x
    circle.center.y = obstacle_wrtGlobal[i][1];             //obstacle_y

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
  ros::init(argc, argv, "sensor_fusion_node");
  // Construct a sensor_fusion object
  SensorFusion deprojecton_obj; // = SensorFusion();
  ros::spin();                    //spin the ros node.
  return 0;
}