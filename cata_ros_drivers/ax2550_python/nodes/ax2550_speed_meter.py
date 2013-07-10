#!/usr/bin/env python
# encoding: utf-8

import roslib; roslib.load_manifest('ax2550_python')
import rospy
from rospy.rostime import Time
from std_msgs.msg import String

from ax2550_python.msg import Encoder

def ax2550SpeedTest():
    speed_test_pub = rospy.Publisher('/cata/speed_test', String)

    # Setup Publisher for publishing voice messages 
    voice_pub = rospy.Publisher('/cata/cata_voice', String)
    
    current_time = rospy.Time.now().secs
    init_time = current_time
    time_difference = 0
    
    test_direction = rospy.get_param('~speed_test_direction', "cw") # cw: clockwise, ccw: counter-clockwise
    max_duration = rospy.get_param('~max_duration', 10) # seconds
    
    test_msg = "end" # just in case
    init_msg = "Beggining speed test"
    if test_direction == "cw": # Turn clockwise
        test_msg = "cw"
        init_msg += " in the clockwise direction!"
    if test_direction == "ccw": # Turn counter-clockwise
        test_msg = "ccw"
        init_msg += " in the counterclockwise direction!"
    
    voice_pub.publish(String(init_msg))

    speed_test_pub.publish(String(test_msg))
    while time_difference < max_duration:
        current_time = rospy.Time.now().secs
        time_difference = current_time - init_time
    
    # Send stop command
    test_msg = "end"
    speed_test_pub.publish(String(test_msg))
    
    end_summary = "Speed test ran for %d seconds. GAME OVER" %time_difference
    rospy.loginfo(end_summary)     

    voice_pub.publish(String(end_summary))
        
if __name__ == '__main__':
    rospy.spin()
    ax2550SpeedTest()
