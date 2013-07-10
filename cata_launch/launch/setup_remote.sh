#!/bin/bash

if  ping -b eth0 -c 1 192.168.2.1 # to Robotics2 ip
then
    export ROS_MASTER_URI=http://192.168.2.10:11311
else
    export ROS_MASTER_URI=http://192.168.1.2:11311
fi

echo $ROS_MASTER_URI;

USER_NAME=$USER

# Change from wlan0 or eth0 to fit your needs!!!
echo "Hello $USER_NAME"
if [ $USER_NAME == 'carlos' ] 
	then	
		WIRELESSIP=`ifconfig eth0  | grep 'inet addr:'| grep -v '127.0.0.1' | cut -d: -f2 | awk '{ print $1}'`
else
	if [ $USER_NAME == 'cata' ] 
		then	
			WIRELESSIP=`ifconfig eth0  | grep 'inet addr:'| grep -v '127.0.0.1' | cut -d: -f2 | awk '{ print $1}'`
		else
			WIRELESSIP=`ifconfig wlan0  | grep 'inet addr:'| grep -v '127.0.0.1' | cut -d: -f2 | awk '{ print $1}'`
	fi
fi

export ROS_IP=$WIRELESSIP ;
echo $ROS_IP;
