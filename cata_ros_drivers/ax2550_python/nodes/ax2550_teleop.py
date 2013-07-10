#!/usr/bin/env python
# encoding: utf-8

"""
ax2550_teleop.py - Used joy stick messages over /joy to send drive commands to the motor controller

Created by William Woodall on 2010-04-13. Modified a lot by Carlos Jrarmillo on 2011-2012
"""
__author__ = "Carlos Jaramillo"
__license__ = "GPL v3"

###  Imports  ###

# ROS imports
import roslib; roslib.load_manifest('ax2550_python')
import rospy

# ROS msg and srv imports
from ax2550_python.srv import Move
from ax2550_python.srv import NavMode
#from joy.msg import Joy # Prior to Electric
from sensor_msgs.msg import Joy # new in Electric
import ax2550_speed_meter

operation_mode = 0 # to toggle between 0 for manual (joystick) mode, 1 for autonomous
speed_sensitivity_factor = 6 # To reduce joystick sensitivity to control speed
# These are powers, to get a speed/(2^factor)
min_speed_sensitivity_factor = 7
max_speed_sensitivity_factor = 1

speed_test_mode = False

#Parameteres:
button_toggler = 0
button_speed_test = 1
button_speed_increaser = 5
button_speed_decreaser = 4

def moveCmdFromJoy(speed, direction):
    """Calls the move srv on ax2550_driver.py"""
    global speed_sensitivity_factor
    
    try:
        factor = 10 * speed_sensitivity_factor
        speed /= factor
        direction /= factor
        moveFnc = rospy.ServiceProxy('move', Move)
        resp1 = moveFnc(speed, direction)
        return resp1.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def navModeCmdFromJoy(toggle):
    """Calls the move srv on ax2550_driver.py"""
    global operation_mode
    
    if(toggle > 0):  # button is pressed
        try:
            shiftMode = rospy.ServiceProxy('joy_mode_switch', NavMode)
            # Toggle modes:
            if operation_mode == 0:
                operation_mode = 1
            else:
                operation_mode = 0

            respButton = shiftMode(operation_mode)
            return respButton.result
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

def changeSpeed(decrease_button, increase_button):
    """Calls the move srv on ax2550_driver.py"""
    global speed_sensitivity_factor, operation_mode, max_speed_sensitivity_factor, min_speed_sensitivity_factor
   
    if(operation_mode == 0): # only work in manual mode
      if(increase_button == 1 and speed_sensitivity_factor > max_speed_sensitivity_factor):  # button to increase sensitivity is pressed
	  speed_sensitivity_factor = speed_sensitivity_factor - 1
      elif (decrease_button == 1  and speed_sensitivity_factor < min_speed_sensitivity_factor): # button to decrease sensitivity is pressed
	  speed_sensitivity_factor = speed_sensitivity_factor + 1 
            
def joystickCallback(data):
    """Called everytime the joystick updates"""
    global button_toggler, button_speed_test, button_speed_increaser, button_speed_decreaser, speed_test_mode
    if data.buttons[button_speed_test] == 1 and speed_test_mode == True:
        # run speed test
        ax2550_speed_meter.ax2550SpeedTest()
    else:    
        #    move(data.axes[1], data.axes[0]) # Game mode
        moveCmdFromJoy(data.axes[1], data.axes[2]) # Speed/Direction Separate control on joystick
        navModeCmdFromJoy(data.buttons[button_toggler]) # Check if this button has been pressed 
        changeSpeed(data.buttons[button_speed_decreaser], data.buttons[button_speed_increaser]) # Modifies joystick's speed sensitivity according to button

def joystickListener():
    """Listens for Joystick signals"""
    global button_toggler, button_speed_test, button_speed_increaser, button_speed_decreaser, speed_test_mode
    
    rospy.init_node('ax2550_teleop', anonymous=True)    
    speed_test_mode = rospy.get_param('~speed_test_mode', False) # To allow for speed testing
    button_toggler = rospy.get_param('~button_as_toggle', 0) # Mapped to a certain button
    button_speed_test = rospy.get_param('~button_for_speed_test', 1) # to start speed test
    button_speed_decreaser = rospy.get_param('~button_speed_decrease', 2)
    button_speed_increaser = rospy.get_param('~button_speed_increase', 3)
    
    s = rospy.Subscriber("joy", Joy, joystickCallback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    joystickListener()
