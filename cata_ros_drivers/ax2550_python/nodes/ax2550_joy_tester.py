#!/usr/bin/env python
# encoding: utf-8

"""
logerror.py - Contains a set of functions that facilitate special error 
logs that account for control code and Hardware Module special cases

Created by Carlos Jaramillo on 2011-04-13.
License GPL v2
"""
__author__ = "Carlos Jaramillo"
__copyright__ = "Copyright (c) Carlos Jaramillo"

###  Imports  ###

# ROS imports
import roslib; roslib.load_manifest('ax2550_python')
import rospy
from rospy.rostime import Time

# ROS msg and srv imports
from std_msgs.msg import String
from ax2550_python.msg import Encoder
from ax2550_python.srv import Move
from ax2550_python.srv import NavMode

# Python Libraries
from threading import Timer, Lock
import time
from time import sleep
import sys
import math

# Peer Libraries
from logerror import logError

###  Classes  ###
class JoyTester(object):
    """This class allows you to test commands from a joystick"""
    def __init__(self):
        """Function called after object instantiation"""
        
        # Set default operation mode
        self.toggleMode = 0 # 0 for manual (joystick) mode, 1 for autonomous
        self.running = True
        
        # Initialize ROS Node
        rospy.init_node('ax2550_joy_tester', anonymous=True)
        
        # Register the Move service with the handleMove function (Usually from data that comes from joystick commands)
        self.move_srv = rospy.Service('move', Move, self.handleMove)

        # Register the NavMode service with the handleNavMode function (based on button being pressed switches between manual and autonomous mode)
        self.joy_nav_mode_srv = rospy.Service('joy_mode_switch', NavMode, self.handleNavMode)
        
        # Register shutdown function
        rospy.on_shutdown(self.shutdown)
        
        # Handle ros srv requests
        rospy.spin()
        
    def handleMove(self, data):
        """Handles the Move srv requests"""
        if self.toggleMode == 0:
            print "Moving with Speed=%0.2f Direction=%0.2f" % (data.speed, data.direction)
        return 0

    def handleNavMode(self, data):
        """Handles the NavMode srv requests that toggle between joystick and autonomous modes"""
        self.toggleMode = data.button_toggle # 0 for manual (joystick) mode, 1 for autonomous
        print "Button pressed changed to mode %i" % self.toggleMode
        return 0

    def shutdown(self):
        """Called when the server shutsdown"""
        self.running = False
            
###  If Main  ###
if __name__ == '__main__':
    JoyTester()




