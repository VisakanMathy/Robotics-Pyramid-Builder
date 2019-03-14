#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

#Initializing the robot into the start position
pub = rospy.Publisher('chatter', Int32, queue_size = 10)
rospy.init_node('Starter_node')
r = rospy.Rate(10)
while not rospy.is_shutdown():
    pub.publish(4)
    r.sleep()
