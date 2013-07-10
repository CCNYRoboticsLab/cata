/* ======================================================================
 * cata_gps.cpp
 *       GPS driver that wraps around ROS_ARIA's Novatel GPS driver
 *
 *  Written by Sebastian Pendola and Carlos Jaramillo, CCNY
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 * ======================================================================
 */

// Ros includes
#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/GPSFix.h>
#include <gps_common/conversions.h>
#include "std_msgs/String.h"

#include "Aria.h"
#include "ArGPS.h"
#include "ArGPSConnector.h"
#include "ArTrimbleGPS.h"
#include "ArTCMCompassDirect.h"

using namespace sensor_msgs;
sensor_msgs::NavSatFix navsat_fix;
gps_common::GPSFix common_fix;

int
main(int argc, char **argv)
{
  //system("clear");
  printf("Novatel Driver - CCNY\n\n");

  printf("-> initializing ROS\n");
  ros::init(argc, argv, "cata_gps");
  ros::NodeHandle n;

  printf("   creating publisher (/fix)");
  ros::Publisher navsat_gps_pub = n.advertise<sensor_msgs::NavSatFix> ("/fix",
      5);
  ros::Publisher common_gps_pub = n.advertise<gps_common::GPSFix> (
      "/fix_common", 5);
  ros::Publisher voice_pub = n.advertise<std_msgs::String> ("/cata_voice", 5);

  ros::Rate loop_rate(10);
  printf("   ROS is ready\n\n");

  printf("-> initializing Aria\n");
  Aria::init();

  ArArgumentParser argParser(&argc, argv);
  argParser.loadDefaultArguments();
  ArSimpleConnector connector(&argParser);
  ArGPSConnector gpsConnector(&argParser);

  if (!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed())
    return 1;

  printf("Connecting to GPS...\n");
  ArGPS *gps = gpsConnector.createGPS(NULL); // .createGPS(&robot)
  if (!gps || !gps->connect())
    {
      printf(
          "Error connecting to GPS device\n(maybe you could try -gpsType, -gpsPort, and/or -gpsBaud command-line arguments.)\n");
      return -1;
    }
  printf("GPS online, ready to begin publishing data...\n");
  std_msgs::String msg;
  msg.data = "GPS is connected and publishing";
  voice_pub.publish(msg);
  ArTime lastReadTime;

  while (ros::ok())
    {
      int r = gps->read();
      if (r & ArGPS::ReadError)
        {
          printf("Warning: non-fatal error reading GPS data.\n");
          ArUtil::sleep(1000);
        }

      if (r & ArGPS::ReadUpdated)
        {
          if (!gps->havePosition())
            {
              navsat_fix.longitude = 0.0f;
              navsat_fix.latitude = 0.0f;
              common_fix.longitude = 0.0f;
              common_fix.latitude = 0.0f;
            }
          else
            {
              navsat_fix.longitude = gps->getLongitude();
              navsat_fix.latitude = gps->getLatitude();
              common_fix.longitude = navsat_fix.longitude;
              common_fix.latitude = navsat_fix.latitude;
            }

          if (!gps->haveLatLonError())
            {
              common_fix.err_vert = 0.0f;
              common_fix.err_horz = 0.0f;
            }
          else
            {
              common_fix.err_vert = gps->getLatLonError().getX();
              common_fix.err_horz = gps->getLatLonError().getY();
            }

          if (!gps->haveAltitude())
            {
              navsat_fix.altitude = 0.0f;
              common_fix.altitude = 0.0f;
            }
          else
            {
              navsat_fix.altitude = gps->getAltitude();
              common_fix.altitude = navsat_fix.altitude;
            }

          if (!gps->haveHDOP())
            {
              ROS_WARN("cata_gps: No HDOP");
              common_fix.hdop = 1e-5f;
            }
          else
            common_fix.hdop = gps->getHDOP();

          if (!gps->haveVDOP())
            {
              ROS_WARN("cata_gps: No VDOP");
              common_fix.vdop = 1e-5f;
            }
          else
            common_fix.vdop = gps->getVDOP();

          // TODO: find appropriate values (these are ok for now)
          // this is a test based on the gpsd_client code for covariance
          navsat_fix.position_covariance[0] = gps->getLatLonError().getX()
              * gps->getLatLonError().getX();
          navsat_fix.position_covariance[4] = gps->getLatLonError().getY()
              * gps->getLatLonError().getY();
          navsat_fix.position_covariance[8] = gps->getAltitudeError()
              * gps->getAltitudeError();
          navsat_fix.position_covariance_type
              = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

          // TODO: try both and fix problem about "filter time older than vo message buffer"
          navsat_fix.header.stamp = ros::Time::now();

          navsat_gps_pub.publish(navsat_fix);
          common_gps_pub.publish(common_fix);
          ros::spinOnce();
          loop_rate.sleep();

          fflush(stdout);
          ArUtil::sleep(500);
          lastReadTime.setToNow();
          continue;
        }
      else
        {
          if (lastReadTime.secSince() >= 5)
            {
              printf(
                  "Warning: haven't recieved any data from GPS for more than 5 seconds!\n");
              msg.data = "I am not receiving data from the GPS";
              voice_pub.publish(msg);
            }
        }
      //ros::spin();
    }

  return 0;
}
