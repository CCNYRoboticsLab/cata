#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <netinet/in.h>
#include "festival/festival.h"
#define BUFFSIZE 255


void voiceCallBack(const std_msgs::String::ConstPtr& msg)
{
	EST_String text(msg->data.c_str());
	festival_say_text(text);
	festival_wait_for_spooler();	
}

int main(int argc, char **argv)
{
	system("clear");
	printf("Cata Voice (ROS) - CCNY 2011\n\n");

	printf("-> initializing ROS topic\n");
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/cata/cata_voice", 10, voiceCallBack);
  	
	printf("-> initializing festival API ...\n");
	int heap_size=210000;
	int load_init_files=1;
	festival_initialize(load_init_files,heap_size);

	ROS_INFO("-> Complete\nCata_Voice is listening to topic 'cata_voice'");
	festival_say_text("cata voice is ready");
	festival_wait_for_spooler();

  	ros::spin();
	return 0;
}


