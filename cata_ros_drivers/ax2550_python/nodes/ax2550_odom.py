#!/usr/bin/env python
# encoding: utf-8

import roslib; roslib.load_manifest('ax2550_python')
import rospy
from rospy.rostime import Time

from ax2550_python.msg import Encoder 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,Point

import tf
import math

odom_pose = None
odom_pub = None
odom_broadcaster = None

wheel_base_width = 0.70 # WHEEL_BASE_LENGTH   = 0.70 # meters (CATA)
# Found out proper encoder resolution by
# counting the pulses output in one revolution...it's better to do pulses_in_10_revs/10 revs) 
encoder_resolution = 1920 #cycles per revolution * 4 (quadrature) = pulses per revolution
wheel_diameter_left = 0.30 #meters (CATA) 
wheel_diameter_right = 0.30 #meters (CATA)
wheel_circum_left = math.pi * wheel_diameter_left
wheel_circum_right = math.pi * wheel_diameter_right
theta=0.0 # heading angle of the robot
x=0.0 # x position
y=0.0 # y posititon
current_time = 0 
previous_time = 0
left_cpr = 0;
right_cpr = 0;


MAX_DBL = 1e+100  

def encoderDataReceived(data):
    """Called when encoder data is received"""
    global odom_pub,odom_pose
    global wheel_base_width, wheel_diameter_left, wheel_diameter_right, wheel_circum_left, wheel_circum_right,encoder_resolution
    global x, y, theta
    global current_time,previous_time
    
    # --------------------------
    # Just for getting the actual PPR:
    # Just for counting (testing) encoders' CPR
    #global left_cpr, right_cpr
    #left_cpr += data.left
    #right_cpr += data.right
    #print "CPR Left = %d" % (left_cpr) # 19195 pulses / 10 revs = 1920 ppr =
    #print "CPR Right = %d" % (right_cpr) # 19185 pulses / 10 revs = 1919 ppr = 
    # -------------------------------------------
    
    current_time = rospy.Time.now()
    time_delta = (current_time - previous_time).to_sec()
    
    # The following computes the linear distance traveled by each wheel
    # distance = (number of pulses read from encoder) * (wheel circumference) / (pulses per revolution)
    left = data.left * wheel_circum_left/encoder_resolution
    right = data.right * wheel_circum_right/encoder_resolution

    # Linear velocities:
    vl = left / time_delta
    vr = right / time_delta
    angular_velocity = (vr-vl)/wheel_base_width;
    linear_velocity_x = (vr+vl)/2;     
    # vy is always 0 for a differential drive
    
#    delta_x = (linear_velocity_x * cos(theta) - vy * sin(theta)) * time_delta;
    delta_x = (linear_velocity_x * math.cos(theta)) * time_delta;
#    delta_y = (linear_velocity_x * sin(theta) + vy * cos(theta)) * time_delta;
    delta_y = (linear_velocity_x * math.sin(theta)) * time_delta;
    delta_theta = angular_velocity * time_delta;

    # Update 2-D position components:
    x += delta_x;
    y += delta_y;
    theta += delta_theta;

    # Save time
    previous_time = current_time
 
    quat = tf.transformations.quaternion_from_euler(0,0,theta)
   
    ### Insert math into Odom msg so it can be published
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    #odom_msg.header.frame_id="odom_combined"
    odom_msg.header.frame_id="odom_wheel_frame"
    odom_msg.pose.pose.position.x = x
    odom_msg.pose.pose.position.y = y
    odom_msg.pose.pose.position.z = 0.0
    odom_msg.pose.pose.orientation.x = quat[0]
    odom_msg.pose.pose.orientation.y = quat[1]
    odom_msg.pose.pose.orientation.z = quat[2]
    odom_msg.pose.pose.orientation.w = quat[3]

    # TODO: fill with own covariance values
    # 6x6 Covariance matrix
    odom_msg.pose.covariance = [1e-5, 0, 0, 0, 0, 0,  # x
			        0, 1e-5, 0, 0, 0, 0,  # y
				0, 0, MAX_DBL, 0, 0, 0,   # z
				0, 0, 0, MAX_DBL, 0, 0,   # x_ang
				0, 0, 0, 0, MAX_DBL, 0,   # y_ang
				0, 0, 0, 0, 0, 1e-3]      # z_ang
    odom_msg.twist.twist.linear.x = linear_velocity_x
#    odom_msg.twist.twist.linear.y = 0.0
#    odom_msg.twist.twist.angular.x = theta_dot
    odom_msg.twist.twist.angular.z = angular_velocity   # Carlos: I think this should be angular velocity on z
    odom_msg.twist.covariance = odom_msg.pose.covariance
 
    odom_pose_msg = PoseStamped()
    odom_pose_msg.header.stamp = rospy.Time.now()
    #odom_pose_msg.header.frame_id="odom_combined"
    odom_pose_msg.header.frame_id="pose_wheel_frame"
    odom_pose_msg.pose.orientation.x = quat[0]
    odom_pose_msg.pose.orientation.y = quat[1]
    odom_pose_msg.pose.orientation.z = quat[2]
    odom_pose_msg.pose.orientation.w = quat[3]
    odom_pose_msg.pose.position=Point(x,y,0)

    ### Publishing Odom_msg
    odom_pub.publish(odom_msg)
    odom_pose.publish(odom_pose_msg)    

def ax2550EncodersListener():
    """Main loop"""
    global odom_pub
    global odom_pose
    global current_time,previous_time
    global odom_broadcaster
    
    rospy.init_node('base_odom', anonymous=True)
    rospy.Subscriber('/cata/motor_control_encoders', Encoder, encoderDataReceived)
        
    odom_pub = rospy.Publisher('/cata/base_odom', Odometry)
    odom_pose = rospy.Publisher('/cata/base_pose', PoseStamped)

    current_time=rospy.Time.now()
    previous_time = current_time
    
    odom_broadcaster = tf.TransformBroadcaster()

    rospy.spin()
    
if __name__ == '__main__':
    ax2550EncodersListener()
