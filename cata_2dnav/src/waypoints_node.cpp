#include <ros/ros.h>
#include "std_msgs/String.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <gps_common/conversions.h>
#include <tf/transform_datatypes.h>
#include <queue>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

class waypoints
{
    struct coords
    {
        double x;
        double y;
    };
  public:
    waypoints(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~waypoints();
    void
    sendGoals();

  private:
    void
    readWaypoints(std::string file);
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    //ros::PUblisher voice_pub_;
    //ros::Publisher voice_pub = n.advertise<std_msgs::String>("/cata_voice", 5);


    std::queue<coords> waypoints_queue_;
    std::string waypoints_file_name_;
    std_msgs::String msg;
    bool has_waypoints_file_;
    bool is_gps_waypoints_;

};

waypoints::waypoints(ros::NodeHandle nh, ros::NodeHandle nh_private) :
  nh_(nh), nh_private_(nh_private)
{
    if (!nh_private.getParam("gps_waypoints", is_gps_waypoints_))
    {
      is_gps_waypoints_ = false;
      ROS_INFO("Using Cartessian waypoints");
    }
  else
    {
      if(is_gps_waypoints_)
	ROS_INFO("Using GPS waypoints");
      else
	ROS_INFO("Using Cartessian waypoints");
    }
    
  if (nh_private.getParam("waypoints_file", waypoints_file_name_))
    {
      //msg.data = "I am reading waypoints from my launch files";
      //voice_pub.publish(msg);
      has_waypoints_file_ = true;
      // Read the waypoints from file
      readWaypoints(waypoints_file_name_);
    }
  else
    {
      //msg.data = "I did not find waypoints in my launch files";
      //voice_pub.publish(msg);
      has_waypoints_file_ = false;
      ROS_WARN("No waypoints file name given as parameter");
    }
   
}

waypoints::~waypoints()
{
  // NADA
}


void waypoints::readWaypoints(std::string file)
{
      // Clear queue
      waypoints_queue_ = std::queue<coords> ();
      
	std::ifstream fh;
	fh.open(file.c_str());
	
	struct coords coordinates;
	char comma;

	while (!fh.eof())
	{
		double lat, lon;
		std::string zone;
		fh >> std::setprecision(18) >> lat; //reads in the double value
		fh.get(comma);
		fh >> std::setprecision(18) >> lon; //reads in the double value

		if(is_gps_waypoints_)
		  gps_common::LLtoUTM(lat, lon, coordinates.y, coordinates.x, zone); // Here y: Northing, x: Easting
		else
		{
		  coordinates.y = lat;
		  coordinates.x = lon;
		}
				
		waypoints_queue_.push(coordinates);
		
#ifdef DEVELOP
		std::cout << "(Lat, Lon):" << std::setprecision(16) << lat << ", " << std::setprecision(16) << lon << std::endl; //for test purposes
		std::cout << "(X, Y):" << std::setprecision(16) << coordinates.x << ", " << std::setprecision(16) << coordinates.y << std::endl; //for test purposes
#endif
    	}
	fh.close();

  	ROS_INFO("Finished reading file %s", file.c_str());
}

void
waypoints::sendGoals()
{
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base_node", true); // TODO: perhaps, it should be only "move_base"!!!

  //wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

  //msg.data = "Starting Navigation";
  //voice_pub.publish(msg);
  move_base_msgs::MoveBaseGoal goal;

  while (!waypoints_queue_.empty())
    {
      //we'll send a goal to the robot to move 1 meter forward
      goal.target_pose.header.frame_id = "base_link";
      goal.target_pose.header.stamp = ros::Time::now();

      // TODO: sort out what is x and y (file reads latitude, longitude)
      goal.target_pose.pose.position.x = waypoints_queue_.front().x;
      goal.target_pose.pose.position.y = waypoints_queue_.front().y;
      double theta = 3.14; // TODO: setting arbitrary heading (yaw)
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      std::cout << "Sending (X, Y):" << std::setprecision(16)
          << goal.target_pose.pose.position.x << ", " << std::setprecision(16)
          << goal.target_pose.pose.position.y << std::endl; //for test purposes

      ac.waitForResult();

      if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          ROS_INFO("waypoint goal complete");
          //msg.data = "waypoint goal complete";
          waypoints_queue_.pop();
        }
      else
        {
          ROS_INFO("failed to complete waypoint goal");
          //msg.data = "failed to complete waypoint goal";
          // if we dont pop here... we should see the robot attempting the same waypoint again
        }
      //voice_pub.publish(msg);
    }
}

int
main(int argc, char** argv)
{
  printf("-> initializing ROS\n");
  ros::init(argc, argv, "waypoints_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  waypoints wp(nh, nh_private);
  wp.sendGoals();
  ros::Rate loop_rate(10);
  ros::spin();

  return 0;
}

