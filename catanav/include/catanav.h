// catanav.h header
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <gps_common/conversions.h>
#include <gps_common/GPSFix.h>
#include <gps_common/GPSStatus.h>
#include <os5000/CompassData.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include <vector>


//void gpsSubscriberCallBack(const gps_common::GPSFix::ConstPtr& gpsData);
void gpsSubscriberCallBack(const sensor_msgs::NavSatFix::ConstPtr& gpsData);
void compassSubscriberCallBack(const os5000::CompassData::ConstPtr& compassData);
void targetSubscriberCallBack(const std_msgs::String::ConstPtr& msg);
void odomSubscriberCallBack(const nav_msgs::Odometry::ConstPtr& odom);
void laserSubscriberCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);

ros::Publisher publisherStatus;
ros::Publisher publisherCmdVel;

double targetYaw, currentYaw, rawYaw;
double currentLatitude, rawLatitude;
double currentLongitude, rawLongitude;
double currentAltitude, rawAltitude;
double gpsVariance;

bool isPathStillValid();
bool isRobotAtTarget();
bool findNewPath();
bool printStatus();
bool syncRobotCompass();
bool syncRobotGPS();
bool pubCmdVelocity();
bool pubCmdVelocity(float x, float y, float z, float ax, float ay, float az);

bool orientateRobot(float yaw);

