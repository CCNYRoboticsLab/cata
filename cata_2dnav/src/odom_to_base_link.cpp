/*
 * odom_to_base_link.cpp
 *
 *  Created on: Jun 5, 2011
 *      Author: carlos
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ros::Publisher odom_pub;

void
ekf_poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  static double prev_theta = 0;
  static double prev_x = 0;
  static double prev_y = 0;
  static double prev_z = 0;
  static ros::Time prev_time = msg->header.stamp;

  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double z = msg->pose.pose.position.z;
  double theta = tf::getYaw(msg->pose.pose.orientation);

  ros::Time current_time = msg->header.stamp;

  //compute odometry in a typical way given the velocities of the robot
  double dt = (current_time - prev_time).toSec();

  double v_x, v_y, v_z, w_z;

  if (dt > 0.0)
    {
      v_x = (x - prev_x) / dt;
      v_y = (y - prev_y) / dt;
      v_z = (z - prev_z) / dt;
      w_z = (theta - prev_theta) / dt;
    }
  else
    {
      v_x = 0;
      v_y = 0;
      v_z = 0;
      w_z = 0;
    }

  prev_theta = theta;
  prev_x = x;
  prev_y = y;
  prev_z = z;
  prev_time = current_time;

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "cata_odom_combined_frame";
  //      odom.child_frame_id = "base_link"; // TODO: not sure

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = z;
  odom.pose.pose.orientation = msg->pose.pose.orientation;

  //set the velocity
  odom.child_frame_id = "base_link";
  // TODO: try to get the correct twist values from the motor controller
  odom.twist.twist.linear.x = v_x;
  odom.twist.twist.linear.y = v_y;
  odom.twist.twist.linear.z = v_z;
  odom.twist.twist.angular.z = w_z;

  odom.twist.covariance = msg->pose.covariance;

  //publish the message
  odom_pub.publish(odom);
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  odom_pub = n.advertise<nav_msgs::Odometry> ("/cata/odom_combined", 50);
  ros::Subscriber ekf_sub = n.subscribe("/robot_pose_ekf/odom_combined", 10,
      ekf_poseCallback);

  ros::Rate loop_rate(10);
  ros::spin();
}
