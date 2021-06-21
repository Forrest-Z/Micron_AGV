#include "includes/MarkerNode.h"
#include <qdebug.h>
#include "std_msgs/String.h"

MarkerNode::MarkerNode(){ }

void MarkerNode::marker_display(){
    sleep(2);
    int argc = 0; char **argv = nullptr;
    ros::init(argc, argv, "marker"/*, ros::init_options::AnonymousName*/);
    n = new ros::NodeHandle();
    pose_sub = new ros::Subscriber();
    *pose_sub = n->subscribe("amcl_pose", 10, &Pose_::poseCallback, &amcl_pose);
    marker_pub = new ros::Publisher;
    *marker_pub = n->advertise<visualization_msgs::Marker>("vis_marker", 5);
    ros::Rate loop_rate(10);
    while (ros::ok()){
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "marker";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::MESH_RESOURCE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.mesh_resource = "file:///home/agv/UI/AGV-UI/arrow2.stl";
//      qDebug() << amcl_pose.pose_.position.x << "  " << amcl_pose.pose_.position.y <<
//          "   " << amcl_pose.pose_.position.z;
      marker.pose.position.x = amcl_pose.pose_.position.x;
      marker.pose.position.y = amcl_pose.pose_.position.y;
      marker.pose.position.z = amcl_pose.pose_.position.z;
      marker.pose.orientation.x = amcl_pose.pose_.orientation.x;
      marker.pose.orientation.y = amcl_pose.pose_.orientation.y;
      marker.pose.orientation.z = amcl_pose.pose_.orientation.z;
      marker.pose.orientation.w = amcl_pose.pose_.orientation.w;

// color, scale, a
      {
      marker.scale.x = 0.003;
      marker.scale.y = 0.003;
      marker.scale.z = 0.003;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      }
      marker.lifetime = ros::Duration();
      marker_pub->publish(marker);
      //ROS_INFO("%f  %f  %f", amclpose.pose.position.x,amclpose.pose.position.y,amclpose.pose.position.z);

      ros::spinOnce();
      loop_rate.sleep();
    }
}
MarkerNode::~MarkerNode(){
    ros::shutdown();
    delete n;
    emit finished();
}
