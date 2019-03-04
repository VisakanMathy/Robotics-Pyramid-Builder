#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

pub = rospy.Publisher('chatter', Int32, queue_size = 10)
rospy.init_node('Starter_node')
r = rospy.Rate(10)
layer= input(int('how many layers?'))
while not rospy.is_shutdown():
    pub.publish(layer)
    r.sleep()
