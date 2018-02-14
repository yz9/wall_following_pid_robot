/*
 * Copyright (c) 2011, Willow Garage, Inc.
 *           (c) 2013, Mike Purvis
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <algorithm>
#include <string>
#include <map>

using visualization_msgs::InteractiveMarker;
using visualization_msgs::InteractiveMarkerControl;
using visualization_msgs::InteractiveMarkerFeedback;

class MarkerServer
{
  public:
    MarkerServer()
      : nh("~"), server("twist_marker_server")
    {
      std::string cmd_vel_topic;

      nh.param<std::string>("link_name", link_name, "/base_link");
      nh.param<std::string>("robot_name", robot_name, "robot");

      if (nh.getParam("linear_scale", linear_drive_scale_map))
      {
        nh.getParam("linear_scale", linear_drive_scale_map);
        nh.getParam("max_positive_linear_velocity", max_positive_linear_velocity_map);
        nh.getParam("max_negative_linear_velocity", max_negative_linear_velocity_map);
      }
      else
      {
        nh.param<double>("linear_scale", linear_drive_scale_map["x"], 1.0);
        nh.param<double>("max_positive_linear_velocity", max_positive_linear_velocity_map["x"],  1.0);
        nh.param<double>("max_negative_linear_velocity", max_negative_linear_velocity_map["x"], -1.0);
      }

      nh.param<double>("angular_scale", angular_drive_scale, 2.2);
      nh.param<double>("max_angular_velocity", max_angular_velocity, 2.2);
      nh.param<double>("marker_size_scale", marker_size_scale, 1.0);

      vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
      createInteractiveMarkers();

      ROS_INFO("[twist_marker_server] Initialized.");
    }

    void processFeedback(
        const InteractiveMarkerFeedback::ConstPtr &feedback);

  private:
    void createInteractiveMarkers();

    ros::NodeHandle nh;
    ros::Publisher vel_pub;
    interactive_markers::InteractiveMarkerServer server;

    std::map<std::string, double> linear_drive_scale_map;
    std::map<std::string, double> max_positive_linear_velocity_map;
    std::map<std::string, double> max_negative_linear_velocity_map;

    double angular_drive_scale;
    double max_angular_velocity;
    double marker_size_scale;

    std::string link_name;
    std::string robot_name;
};

void MarkerServer::processFeedback(
    const InteractiveMarkerFeedback::ConstPtr &feedback )
{
  geometry_msgs::Twist vel;

  // Handle angular change (yaw is the only direction in which you can rotate)
  double yaw = tf::getYaw(feedback->pose.orientation);
  vel.angular.z = angular_drive_scale * yaw;
  vel.angular.z = std::min(vel.angular.z,  max_angular_velocity);
  vel.angular.z = std::max(vel.angular.z, -max_angular_velocity);

  if (linear_drive_scale_map.find("x") != linear_drive_scale_map.end())
  {
    vel.linear.x = linear_drive_scale_map["x"] * feedback->pose.position.x;
    vel.linear.x = std::min(vel.linear.x, max_positive_linear_velocity_map["x"]);
    vel.linear.x = std::max(vel.linear.x, max_negative_linear_velocity_map["x"]);
  }
  if (linear_drive_scale_map.find("y") != linear_drive_scale_map.end())
  {
    vel.linear.y = linear_drive_scale_map["y"] * feedback->pose.position.y;
    vel.linear.y = std::min(vel.linear.y, max_positive_linear_velocity_map["y"]);
    vel.linear.y = std::max(vel.linear.y, max_negative_linear_velocity_map["y"]);
  }
  if (linear_drive_scale_map.find("z") != linear_drive_scale_map.end())
  {
    vel.linear.z = linear_drive_scale_map["z"] * feedback->pose.position.z;
    vel.linear.z = std::min(vel.linear.z, max_positive_linear_velocity_map["z"]);
    vel.linear.z = std::max(vel.linear.z, max_negative_linear_velocity_map["z"]);
  }

  vel_pub.publish(vel);

  // Make the marker snap back to robot
  server.setPose(robot_name + "_twist_marker", geometry_msgs::Pose());

  server.applyChanges();
}

void MarkerServer::createInteractiveMarkers()
{
  // create an interactive marker for our server
  InteractiveMarker int_marker;
  int_marker.header.frame_id = link_name;
  int_marker.name = robot_name + "_twist_marker";
  int_marker.description = "twist controller for " + robot_name;
  int_marker.scale = marker_size_scale;

  InteractiveMarkerControl control;

  control.orientation_mode = InteractiveMarkerControl::FIXED;

  if (linear_drive_scale_map.find("x") != linear_drive_scale_map.end())
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }
  if (linear_drive_scale_map.find("y") != linear_drive_scale_map.end())
  {
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }
  if (linear_drive_scale_map.find("z") != linear_drive_scale_map.end())
  {
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);



  server.insert(int_marker, boost::bind(&MarkerServer::processFeedback, this, _1));

  server.applyChanges();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_server");
  MarkerServer server;

  ros::spin();
}
