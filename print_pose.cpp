#include "math.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

void poseCallback(const turtlesim::Pose::ConstPtr& pose_message); // To be implemented further down

turtlesim::Pose current_pose;

 
int main(int argc, char **argv) {

  ros::init(argc, argv, "print_pose_node");
  ros::NodeHandle node_handle;

  ros::Publisher twist_pub = node_handle.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
  ros::Subscriber pose_sub = node_handle.subscribe("/turtle1/pose/", 10, poseCallback);
  
  geometry_msgs::Twist twist_msg;

  ros::Rate rate(60);

 ROS_INFO("Entering while loop");
  while (ros::ok()) { 
    //do something
    ros::spinOnce();
    rate.sleep();  
   }
  ROS_INFO("Shutdown detected. Exiting");
  
  return 0; 

}

void poseCallback(const turtlesim::Pose::ConstPtr& pose_message) {
  ROS_INFO("Pose x = %f", pose_message->x);
  ROS_INFO("Pose y = %f", pose_message->y);
  ROS_INFO("Pose theta = %f", pose_message->theta);
}

