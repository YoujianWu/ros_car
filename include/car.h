//
// Created by kook on 2/25/25.
//

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

namespace ros_car
{
// state
enum CarMotion
{
  STOP,
  FORWARD,
  BACKWARD,
  TURN_LEFT,
  REVERSE_LEFT,
  TURN_RIGHT,
  REVERSE_RIGHT,
};

// variables
ros::Time last_time_;
ros::Publisher cmd_pub_;
ros::Subscriber odom_sub_;
geometry_msgs::Twist cmd_vel_;

// functions
void updateMotion(const CarMotion& motion);
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
};  // namespace ros_car