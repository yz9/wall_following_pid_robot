#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <cmath>

#include <wall_following_assignment/pid.h>

ros::Publisher cmd_pub;
double desired_distance_from_wall = 0.0; // in meters
double forward_speed = 0.0;              // in meters / sec

void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  geometry_msgs::Twist cmd;
  cmd.linear.x = forward_speed;  // forward speed is fixed
    
  // Populate this command based on the distance to the closest
  // object in laser scan. I.e. compute the cross-track error
  // as mentioned in the PID slides.

  // You can populate the command based on either of the following two methods:
  // (1) using only the distance to the closest wall
  // (2) using the distance to the closest wall and the orientation of the wall
  //
  // If you select option 2, you might want to use cascading PID control. 
  
  // cmd.angular.z = ???
  cmd_pub.publish(cmd);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "wall_follower_node");
  ros::NodeHandle nh("~");

  // Getting params before setting up the topic subscribers
  // otherwise the callback might get executed with default
  // wall following parameters
  nh.getParam("forward_speed", forward_speed);
  nh.getParam("desired_distance_from_wall", desired_distance_from_wall);

  // todo: set up the command publisher to publish at topic '/husky_1/cmd_vel'
  // using geometry_msgs::Twist messages
  // cmd_pub = ??

  // todo: set up the laser scan subscriber
  // this will set up a callback function that gets executed
  // upon each spinOnce() call, as long as a laser scan
  // message has been published in the meantime by another node
  // ros::Subscriber laser_sub = ??
  
  ros::Rate rate(50);
  // this will return false on ctrl-c or when you call ros::shutdown()
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
  
  return 0;
}
   
