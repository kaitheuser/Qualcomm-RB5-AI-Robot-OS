#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include "april_detection.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/PoseStamped.h"

ros::Publisher pose_pub;
ros::Subscriber image_sub;
AprilDetection det;

void imageCallback(const sensor_msgs::Image::ConstPtr& msg){
  ROS_INFO("image received");
  cv_bridge::CvImagePtr img_cv = cv_bridge::toCvCopy(msg);

  // run detection
  det.processImage(img_cv->image);

  geometry_msgs::PoseStamped pose_msg;
  pose_pub.publish(pose_msg);
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "april_detection");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  pose_pub = n.advertise<geometry_msgs::PoseStamped>("/april_poses", 10); 
  image_sub = n.subscribe("/camera_0", 1, imageCallback);
  
  ros::spin();
  return 0;
  
}