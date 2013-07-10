#include <ros/ros.h>
#include "std_msgs/String.h"
//#include <move_base_msgs/MoveBaseAction.h>
//#include <actionlib/client/simple_action_client.h>
#include <gps_common/conversions.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <queue>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>

//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class waypoints
{
    struct lat_and_long
    {
        double latitude;
        double longitude;
    };
  public:
    waypoints(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~waypoints();
    void
    sendGoals();

  private:
    void
    readWaypoints(std::string file, std::queue<lat_and_long> &waypoints);
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    //ros::PUblisher voice_pub_;
    //ros::Publisher voice_pub = n.advertise<std_msgs::String>("/cata_voice", 5);


    std::queue<lat_and_long> waypointsQueue;
    std::string waypoints_file_name_;
    std_msgs::String msg;
    bool has_waypoints_file;

    ros::Publisher goal_pub;

};

waypoints::waypoints(ros::NodeHandle nh, ros::NodeHandle nh_private) :
  nh_(nh), nh_private_(nh_private)
{
  if (nh_private.getParam("waypoints_file", waypoints_file_name_))
    {
      //msg.data = "I am reading waypoints from my launch files";
      //voice_pub.publish(msg);
      has_waypoints_file = true;
      // Read the waypoints from file
      readWaypoints(waypoints_file_name_, waypointsQueue);
      goal_pub = nh.advertise<move_base_msgs::MoveBaseActionGoal> ("/move_base/goal",
          1);
    }
  else
    {
      //msg.data = "I did not find waypoints in my launch files";
      //voice_pub.publish(msg);
      has_waypoints_file = false;
      ROS_WARN("No waypoints file name given as parameter");
    }
}

waypoints::~waypoints()
{
  // NADA
}

void
waypoints::readWaypoints(std::string file, std::queue<lat_and_long> &wpoints)
{
  std::ifstream fh;
  fh.open(file.c_str());

  struct lat_and_long gps_data;
  char comma;

  while (!fh.eof())
    {
      double lat, lon;
      std::string zone;
      fh >> std::setprecision(18) >> lat; //reads in the double value
      fh.get(comma);
      fh >> std::setprecision(18) >> lon; //reads in the double value


      gps_common::LLtoUTM(lat, lon, gps_data.latitude, gps_data.longitude, zone);
      wpoints.push(gps_data);

      //		#ifdef DEVELOP
      std::cout << "Lat/Long" << std::setprecision(16) << lat << ", "
          << std::setprecision(16) << lon << std::endl; //for test purposes
      std::cout << "(x,y)" << std::setprecision(16) << gps_data.latitude
          << ", " << std::setprecision(16) << gps_data.longitude << std::endl; //for test purposes
      //		#endif
    }
  fh.close();

  ROS_INFO("Finished reading file %s", file.c_str());
}

void
waypoints::sendGoals()
{
  move_base_msgs::MoveBaseActionGoal current_goal;

  while (!waypointsQueue.empty())
    {
      double theta = 3.14159 / 4; // 45 degrees TODO: arbitrary heading for testing
      //we'll send a goal to the robot to move 1 meter forward
      current_goal.header.frame_id = "/cata_odom_combined_frame";
      current_goal.header.stamp = ros::Time::now();

      geometry_msgs::PoseStamped goal_pose;
      goal_pose.header.frame_id = "/cata_odom_combined_frame";
      goal_pose.header.stamp = ros::Time::now();
      goal_pose.pose.position.x = waypointsQueue.front().longitude;
      goal_pose.pose.position.y = waypointsQueue.front().latitude;
      goal_pose.pose.position.z = 0;
      goal_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

      //		ROS_INFO("Sending goal");
      //		ac.sendGoal(goal);
      //		ac.waitForResult();
      //		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      //		{
      //			ROS_INFO("waypoint goal complete");
      //msg.data = "waypoint goal complete";
      //			waypointsQueue.pop();
      //		}
      //		else
      //		{
      //			ROS_INFO("failed to complete waypoint goal");
      //			//msg.data = "failed to complete waypoint goal";
      //			// if we dont pop here... we should see the robot attempting the same waypoint again
      //		}
      //		voice_pub.publish(msg);
      current_goal.goal.target_pose = goal_pose;
      goal_pub.publish(current_goal);

    }
}

int
main(int argc, char** argv)
{
  printf("-> initializing ROS\n");
  ros::init(argc, argv, "cata_goal");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  waypoints wp(nh, nh_private);
  wp.sendGoals();
  ros::Rate loop_rate(1000);
  ros::spin();

  return 0;
}

