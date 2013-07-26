#!/usr/bin/env python
# encoding: utf-8
"""
Created on May 20, 2011
Updated on May 23, 2011
@author: Carlos Chinchilla

This program will communicate with an Arduino UNO board through serial connection.
It will send a signal that will toggle the light mode of the CATA Safety light.

message type: char
board name: "arduino"

board address on CATAs ToughBook: "/dev/arduino_uno"

Mode messages
Autonomous send: char 'a'
Radio-Controlled send: char 'r'
Stand-by send: char 's'
Off send: char 'o' 
"""
__author__ = "Carlos Chinchilla"
__copyright__ = "Copyright (c) CATA Team (CCNY Robotics Lab)"
__ROSified_by__ = "Carlos Jaramillo"

###  Imports  ###

# ROS imports
import roslib; roslib.load_manifest('ax2550_python')
import rospy

# ROS msg and srv imports
from ax2550_python.msg import LightMode

# Python libraries
import serial
import time


###  Classes  ###
class SafetyLight(object):
    """This class allows you to control an ax2550 motor controller"""
    def __init__(self, serial_port=None):
        """Function called after object instantiation"""
        
        # Make a list of usb serial locations to test connection IFF it is unkown
        self.locations = ['/dev/arduino_uno','/dev/ttyACM0','/dev/ttyACM1','/dev/ttyACM3','/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3', '/dev/ttyS0','/dev/ttyS1','/dev/ttyS2','/dev/ttyS3','/dev/tty.usbmodem411'] 
       
        rospy.init_node('arduino_safety_light', anonymous=True)
        
        #Parameteres:
        # Get the serial port name
        self.device = serial_port or rospy.get_param('~serial_port', '/dev/arduino_uno')
        
        #    for device in locations:  
        try:
            # Connect to board
            self.arduino = serial.Serial(self.device, 9600)  
        except:  
            print "SAFETY LIGHT: Failed to connect to", self.device  
    
        print "SAFETY LIGHT: Connected to arduino on", self.device
        # wait for Arduino board to respond
        time.sleep(3)
        self.arduino.write('r')  # standby mode light
        
        #Listens for Autonomous Mode indicator signal
        rospy.Subscriber("/cata/navigation_mode", LightMode, self.lightCallback, queue_size=1)
      
        # Register shutdown function
        rospy.on_shutdown(self.shutdown)

        # Handle ros requests
        rospy.spin()


    def shutdown(self):
        """Called when the node is shutsdown"""
        self.arduino.write('s') # put light in standby mode        
        self.arduino.close()
        
    def lightCallback(self, data):
        """Called everytime the autonomous mode updates"""
        navigation_mode = data.autonomous # True for autonomous mode, False for manual mode 
        
        if navigation_mode == 1: # autonomous
            self.arduino.write('a')  # flashing light
        elif navigation_mode == 0: # manual
            self.arduino.write('r')  # solid light
        # Handles the NavMode message from the subscribe topic"""
        return 0
# end class SafetyLight   

if __name__ == '__main__':
    SafetyLight()
