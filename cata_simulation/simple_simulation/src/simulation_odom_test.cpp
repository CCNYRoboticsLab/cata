// odom_test.cpp
// ======================================================================
// Assignment # 1 - Estimating robot’s position and ICC from differential drive
//
// Written by: Carlos Jaramillo
// G3300 Advanced Mobile Robotics (Spring 2011, Prof. Jizhong Xiao)
//
// ======================================================================

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#define DEBUG   //Debugging flag
//#define SAVE_RESULTS_TO_FILE

const double PI         = 3.14159265;
const double PI_over_2  = PI / 2.0;
const double DEG_TO_RAD = PI / 180.0;
const double RAD_TO_DEG = 180.0 / PI;
const std::string node_name = "odom_test";
const std::string odom_topic_name = "odom";
const std::string marker_topic_name = "ICC_marker";

void drawICC(ros::Publisher &marker_pub, double x, double y, double ICCx, double ICCy);
void drawRobot(ros::Publisher &marker_pub, double base_lenght, double x, double y, geometry_msgs::Quaternion odom_quat);

int main(int argc, char** argv){
  ros::init(argc, argv, node_name );

  int caseNumber = 1; // Default to case 1
  // Case 1: R1=R2=5cm, Wl = Wr =5sin(3t)
  // Case 2: R1=R2=5cm, Wl =5sin(3t), Wr =4sin(3t+1)
  if(argc == 2)
    caseNumber = atoi(argv[1]);

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>(odom_topic_name, 50);
  ros::Publisher odom_subscriber = n.subscribe<nav_msgs::Odometry>(odom_topic_name, 50);
  tf::TransformBroadcaster odom_broadcaster;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>(marker_topic_name, 1);

  // Navigation (2-D position) of a differential-drive mobile robot
  // Robot metric dimensions:
  double L = 0.20; // distance along the axle between the centers of the two wheels (known as B in Hwk # 1)
  double rl=0.05, rr=0.05; // radius of the two wheels
  double max_sim_time = 20.0; // seconds
  double vl=0.0, vr=0.0; // the linear speed of the two wheels (left, right)
  double wl=0.0, wr=0.0; // the angular rate (or angular speed) of the two wheels (left, right)
  double R; // Instantaneous curvature radius (ICR) of the robot trajectory (distance from ICC to the midpoint between the two wheels).
  double ICCx, ICCy; // Instantaneous Center of Curvature

  // Assume the robot starts from the origin.
  double x = 0.0;
  double y = 0.0;
  double theta = DEG_TO_RAD*90.0;  // The mobile robot is facing Y direction

  // Assume the robot has 0 initial velocities
  double vx = 0.0;      // Only this one exists for a differential robot
  double vy = 0.0;
  double vth = 0.0;     // angular velocity on theta (of the robot)

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  double simulation_time = 0.0;

  double freq = 10.0; // Hz
  ros::Rate r(freq);     // Publish n times every second

#ifdef SAVE_RESULTS_TO_FILE
  FILE *pFile;
  char chResultsFileName[50];
  sprintf(chResultsFileName, "time_x_y_%.0fHz-%.1f.csv",
                    freq, ros::Time::now().toSec());
  if ((pFile = fopen(chResultsFileName, "wb")) == NULL)
    {
      printf("saveResultsToFile: Can't create %s\n",
          chResultsFileName);
      exit(0);
    }
    fprintf(pFile, "Time(s),x(m),y(m),Theta(deg)\r\n");
    fprintf(pFile, "%.2f,%.2f,%.2f,%.2f\r\n", simulation_time, x, y, theta*RAD_TO_DEG);
#endif

  while((simulation_time <= max_sim_time) && n.ok()){
    current_time = ros::Time::now();
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    simulation_time += dt;

    switch (caseNumber) {
      case 1: // Case 1: R1=R2=5cm, Wl = Wr =5sin(3t)
        //compute linear speeds of the wheels from their angular speeds:
        wl = wr = 5*sin(3*simulation_time);
        vl = rl*wl;
        vr = rr*wr;
#ifdef DEBUG
        ROS_INFO("Time: %.2f s", simulation_time);
        ROS_INFO("L: wl= %+.3f rad/s, vl= %+.3f m/s", wl, vl);
        ROS_INFO("R: wr= %+.3f rad/s, vr= %+.3f m/s", wr, vr);
#endif
        break;
      case 2:   // Case 2: R1=R2=5cm, Wl =5sin(3t), Wr =4sin(3t+1)
        //compute linear speeds of the wheels from their angular speeds:
        wl = 5*sin(3*simulation_time);
        wr = 4*sin(3*simulation_time+1);
        vl = rl*wl;
        vr = rr*wr;
#ifdef DEBUG
        ROS_INFO("Time: %.2f s", simulation_time);
        ROS_INFO("L: wl= %+.3f rad/s, vl= %+.3f m/s", wl, vl);
        ROS_INFO("R: wr= %+.3f rad/s, vr= %+.3f m/s", wr, vr);
#endif
        break;
      default:

        break;
    }

    vth = (vr-vl)/L;
    vx = (vr+vl)/2;     // vy is always 0 for a differential drive

    double delta_x = (vx * cos(theta) - vy * sin(theta)) * dt;
    double delta_y = (vx * sin(theta) + vy * cos(theta)) * dt;
    double delta_th = vth * dt;

    // Update 2-D position components:
    x += delta_x;
    y += delta_y;
    theta += delta_th;

    //+++++++++++++++++++++++++++++++++++++++++++++++++
    // compute R: instantaneous curvature radius (ICR)
    if(vth != 0.0)// avoid division by zero in cases where there is only translation velocity
      R = vx / vth;
    else
      R = INFINITY;  // multiplied by vx to take care of sign

    // Compute ICC point
    ICCx = x+R*cos(theta+PI_over_2);
    ICCy = y+R*sin(theta+PI_over_2);

    if(vth != 0.0)// when ICC is not infinity
      drawICC(marker_pub, x, y, ICCx, ICCy);
    //+++++++++++++++++++++++++++++++++++++++++++++++++

    ROS_INFO("Position: (%+.3f,%+.3f) m at %.2f s", x, y, simulation_time);
    ROS_INFO("ICR = %.2f m", R);
    ROS_INFO("ICC: (%+.2f, %+.2f) m", ICCx, ICCy);


    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    drawRobot(marker_pub, L, x, y, odom_quat);

#ifdef SAVE_RESULTS_TO_FILE
    fprintf(pFile, "%.2f,%.2f,%.2f,%.2f\r\n", simulation_time, x, y, theta*RAD_TO_DEG);
#endif

    last_time = current_time;
    r.sleep();
  }

#ifdef SAVE_RESULTS_TO_FILE
  fclose(pFile);
#endif

  ROS_INFO("Last Position: (%+.3f,%+.3f) m at %.2f s", x, y, simulation_time);
}

void drawICC(ros::Publisher &marker_pub, double x, double y, double ICCx, double ICCy)
{
  visualization_msgs::Marker ICC_marker;
  visualization_msgs::Marker text_marker;
  visualization_msgs::Marker ICR_marker;
  visualization_msgs::Marker robot_base_marker;
      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      ICC_marker.header.frame_id = "odom";
      ICC_marker.header.stamp = ros::Time::now();
      text_marker.header.frame_id = "odom";
      text_marker.header.stamp = ros::Time::now();
      ICR_marker.header.frame_id = "odom";
      ICR_marker.header.stamp = ros::Time::now();
      robot_base_marker.header.frame_id = "odom";
      robot_base_marker.header.stamp = ros::Time::now();
  // %EndTag(MARKER_INIT)%

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
  // %Tag(NS_ID)%
      ICC_marker.ns = node_name;
      ICC_marker.id = 0;
      text_marker.ns = node_name;
      text_marker.id = 1;
      ICR_marker.ns = node_name;
      ICR_marker.id = 2;
  // %EndTag(NS_ID)%

      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  // %Tag(TYPE)%
      ICC_marker.type = visualization_msgs::Marker::SPHERE;
      text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      ICR_marker.type = visualization_msgs::Marker::LINE_LIST;
  // %EndTag(TYPE)%

      // Set the marker action.  Options are ADD and DELETE
  // %Tag(ACTION)%
      ICC_marker.action = visualization_msgs::Marker::ADD;
      text_marker.action = visualization_msgs::Marker::ADD;
      ICR_marker.action = visualization_msgs::Marker::ADD;
  // %EndTag(ACTION)%

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  // %Tag(POSE)%
      ICC_marker.pose.position.x = ICCx;
      ICC_marker.pose.position.y = ICCy;
      ICC_marker.pose.position.z = 0;
      ICC_marker.pose.orientation.x = 0.0;
      ICC_marker.pose.orientation.y = 0.0;
      ICC_marker.pose.orientation.z = 0.0;
      ICC_marker.pose.orientation.w = 1.0;
      text_marker.pose.position.x = ICCx;
      text_marker.pose.position.y = ICCy - 0.04;
      text_marker.pose.position.z = 0;
      text_marker.pose.orientation.x = 0.0;
      text_marker.pose.orientation.y = 0.0;
      text_marker.pose.orientation.z = 0.0;
      text_marker.pose.orientation.w = 1.0;

      geometry_msgs::Point p;
      p.x = x;
      p.y = y;
      p.z = 0.0;
      // The line list needs two points for each line
      ICR_marker.points.push_back(p);
      p.x = ICCx;
      p.y = ICCy;
      ICR_marker.points.push_back(p);
  // %EndTag(POSE)%

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
  // %Tag(SCALE)%
      ICC_marker.scale.x = 0.03;
      ICC_marker.scale.y = 0.03;
      ICC_marker.scale.z = 0.03;
      text_marker.scale.x = 0.035;
      text_marker.scale.y = 0.035;
      text_marker.scale.z = 0.035;
      // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
      ICR_marker.scale.x = 0.01;
  // %EndTag(SCALE)%

      // Set the color -- be sure to set alpha to something non-zero!
  // %Tag(COLOR)%
      ICC_marker.color.r = 0.5f;
      ICC_marker.color.g = 0.0f;
      ICC_marker.color.b = 0.5f;
      ICC_marker.color.a = 1.0;
      text_marker.color.r = 0.5f;
      text_marker.color.g = 0.0f;
      text_marker.color.b = 0.5f;
      text_marker.color.a = 1.0;
      ICR_marker.color.r = 1.0;
      ICR_marker.color.a = 1.0;
  // %EndTag(COLOR)%

  // %Tag(LIFETIME)%
      ICC_marker.lifetime = ros::Duration(1.0);
      text_marker.lifetime = ros::Duration(1.0);
      ICR_marker.lifetime = ros::Duration(1.0);
  // %EndTag(LIFETIME)%

  // %Tag(TEXT)%
      std::string marker_name = "ICC";
      text_marker.text = marker_name;
  // %EndTag(TEXT)%

      // Publish the marker
  // %Tag(PUBLISH)%
      marker_pub.publish(ICC_marker);
      marker_pub.publish(text_marker);
      marker_pub.publish(ICR_marker);
  // %EndTag(PUBLISH)%

}

void drawRobot(ros::Publisher &marker_pub, double base_lenght, double x, double y, geometry_msgs::Quaternion odom_quat)
{
  visualization_msgs::Marker robot_base_marker;
      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      robot_base_marker.header.frame_id = "odom";
      robot_base_marker.header.stamp = ros::Time::now();
  // %EndTag(MARKER_INIT)%

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
  // %Tag(NS_ID)%
      robot_base_marker.ns = node_name;
      robot_base_marker.id = 3;
  // %EndTag(NS_ID)%

      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  // %Tag(TYPE)%
      robot_base_marker.type = visualization_msgs::Marker::CUBE;
  // %EndTag(TYPE)%

      // Set the marker action.  Options are ADD and DELETE
  // %Tag(ACTION)%
      robot_base_marker.action = visualization_msgs::Marker::ADD;
  // %EndTag(ACTION)%

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  // %Tag(POSE)%
      robot_base_marker.pose.position.x = x;
      robot_base_marker.pose.position.y = y;
      robot_base_marker.pose.position.z = 0;
      robot_base_marker.pose.orientation = odom_quat;

  // %EndTag(POSE)%

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
  // %Tag(SCALE)%
      // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
      robot_base_marker.scale.x = base_lenght;
      robot_base_marker.scale.y = base_lenght/2.0;
      robot_base_marker.scale.z = 0.0;
  // %EndTag(SCALE)%

      // Set the color -- be sure to set alpha to something non-zero!
  // %Tag(COLOR)%
      robot_base_marker.color.b = 1.0;
      robot_base_marker.color.a = 1.0;
  // %EndTag(COLOR)%

  // %Tag(LIFETIME)%
      robot_base_marker.lifetime = ros::Duration(0);
  // %EndTag(LIFETIME)%

      // Publish the marker
  // %Tag(PUBLISH)%
      marker_pub.publish(robot_base_marker);
  // %EndTag(PUBLISH)%

}

