//
// Created by kook on 2/25/25.
//

#include "car.h"

using namespace ros_car;

void ros_car::updateMotion(const CarMotion& motion)
{
  if (motion == CarMotion::STOP)
  {
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.angular.z = 0.0;
  }
  if (motion == CarMotion::FORWARD)
  {
    cmd_vel_.linear.x = 0.5;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.angular.z = 0.0;
  }
  if (motion == CarMotion::BACKWARD)
  {
    cmd_vel_.linear.x = -0.5;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.angular.z = 0.0;
  }
  if (motion == CarMotion::TURN_LEFT)
  {
    cmd_vel_.linear.x = 0.5;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.angular.z = 0.5;
  }
  if (motion == CarMotion::TURN_RIGHT)
  {
    cmd_vel_.linear.x = 0.5;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.angular.z = -0.5;
  }
  if (motion == CarMotion::REVERSE_LEFT)
  {
    cmd_vel_.linear.x = -0.5;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.angular.z = -0.5;
  }
  if (motion == CarMotion::REVERSE_RIGHT)
  {
    cmd_vel_.linear.x = -0.5;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.angular.z = 0.5;
  }
}

void ros_car::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double z = msg->pose.pose.position.z;

  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;

  double Vx = msg->twist.twist.linear.x;
  double Vy = msg->twist.twist.linear.y;
  double Vz = msg->twist.twist.angular.z;

  ROS_INFO("Velocity: Vx=%f, Vy=%f, Vz=%f", Vx, Vy, Vz);
  ROS_INFO("Position: x=%f, y=%f, z=%f", x, y, z);
  ROS_INFO("Orientation: qx=%f, qy=%f, qz=%f, qw=%f", qx, qy, qz, qw);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "car");
  ros::NodeHandle nh("~");

  odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/odom", 10, &odom_callback);
  cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  CarMotion motion = CarMotion::STOP;

  ros::Rate loop_rate(100);
  // loop to process callback and publish msgs.
  while (ros::ok())
  {
    // change motion randomly
    ros::Time current_time = ros::Time::now();
    if ((current_time - last_time_).toSec() > 2)
    {
      motion = static_cast<CarMotion>((motion + 1) % 7);
      last_time_ = current_time;
    }
    // update motion and publish cmd
    updateMotion(motion);
    cmd_pub_.publish(cmd_vel_);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}