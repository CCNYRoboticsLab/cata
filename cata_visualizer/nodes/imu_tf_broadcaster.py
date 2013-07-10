#!/usr/bin/env python  

"""
imu_tf_broadcaster.py - Simple transform broadcaster for an IMU message onto an arbitrary reference frame (e.g. world)
Created by Carlos Jaramillo on 2012-06-15.
"""
__author__ = "Carlos Jaramillo"
__copyright__ = "Copyright (c) GPL v3 Carlos Jaramillo"

###  Imports  ###

# ROS imports
import roslib
roslib.load_manifest('cata_visualizer')
import rospy

# ROS messages imports
import tf
from sensor_msgs.msg import Imu

###  Classes  ###
class ImuTFer(object):
    """This class broadcast a simple, arbitrary transform for an IMU message in reference to some frame target"""
    def __init__(self):
        """Function called after object instantiation"""
        
        # Initialize ROS Node
        rospy.init_node('imu_tf_broadcaster')

       # Get parameters
        self.topic_name = rospy.get_param('~topic', '/imu/data')
        self.reference_frame_name = rospy.get_param('~reference_frame', '/world')
        self.z_offset = rospy.get_param('~z_offset', 1.0)
        
        # Subscribe to the imu topic
        rospy.Subscriber(self.topic_name, Imu, self.handle_imu_orientation)
        
        rospy.spin()
        
    def handle_imu_orientation(self, msg):
        """Callback for the subscriber"""
        orientation_as_tuple = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, self.z_offset),
                     orientation_as_tuple,
                     rospy.Time.now(),
                     msg.header.frame_id,
                     self.reference_frame_name)

# end class AX2550    

###  If Main  ###
if __name__ == '__main__':
    ImuTFer()
