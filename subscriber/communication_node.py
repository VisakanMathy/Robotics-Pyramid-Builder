#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32, String, Int64MultiArray, MultiArrayDimension, Int64
import time
global move_left
global move_right
global topic_number 


topic_number = 0
move_left = [6,8] #This is the 'left position' of the left arm
move_right = [5,7] #This is the 'right position' of the right arm

def callbackleft(data):
	global move_left 
	move_left = data.data

def callbackright(data):
	global move_right
	move_right = data.data

def callback(data):
	global topic_number #Topic numbers are used in the communication node
	global move_left
	global move_right
	
	#Messages are published to the 'hub' to state if the robot arms are ready to move
	pub = rospy.Publisher('do_shit{}'.format(topic_number), Int64MultiArray, queue_size = 10)
	msg = Int64MultiArray()
	msg.layout.dim = [MultiArrayDimension('',2,1)]
	r = rospy.Rate(10)
	start = time.time()
	print('here')

	while time.time()-start < 5:
		print(msg.data)
		if data == 1: 			#Position 1: left picked, right moving to pick
			msg.data = [1,topic_number] 	#This message indicates the position and topic number
			pub.publish(msg) 		#The message is published to the communication node
		elif data== 2: 			#Position 2: right picked, left moving to place
			msg.data = [2,topic_number]
			pub.publish(msg)
		elif data== 3:			#Position 3: left placed, right moving to place
			msg.data = [3,topic_number]
			pub.publish(msg)
		elif data == 4:			#Position 4: right placed, left moving to pick
			msg.data = [4,topic_number]
			pub.publish(msg)
	move_right = [5,7]
	move_left = [6,8]
		

def listener(): #Function to check whether the arms have completed their movements
	global move_left
	global move_right
	global topic_number
	
	while not rospy.is_shutdown():
		'''
		rospy.Subscriber('move_left_arm_hub{}'.format(topic_number+1), Int64MultiArray, callbackleft)
		rospy.Subscriber('move_right_arm_hub{}'.format(topic_number+1), Int64MultiArray, callbackright)
		'''
		if topic_number%2 == 0: #Switch the order of left and right arms for even and odd topic numbers
			msg_left = rospy.wait_for_message('move_left_arm_hub{}'.format(topic_number+1), Int64MultiArray)
			msg_right= rospy.wait_for_message('move_right_arm_hub{}'.format(topic_number+1), Int64MultiArray)
		else:
			msg_right = rospy.wait_for_message('move_right_arm_hub{}'.format(topic_number+1), Int64MultiArray)
			msg_left =  rospy.wait_for_message('move_left_arm_hub{}'.format(topic_number+1),Int64MultiArray)

		move_left = msg_left.data
		move_right =msg_right.data

		if move_left[1] == topic_number and move_right[1] == topic_number: #If BOTH arms have completed their movements
			topic_number +=1 #The topic number increases by one in every iteration to make sure the listener is checking for the latest information
			print('move right {}'.format(move_right))
			print('move left {}'. format(move_left))
			callback(move_left[0])

			
if __name__ == '__main__':
	rospy.init_node("hub")
	pub = rospy.Publisher('do_shit{}'.format(topic_number), Int64MultiArray, queue_size = 10)
	msg = Int64MultiArray()
	msg.layout.dim = [MultiArrayDimension('',2,1)]

	r = rospy.Rate(10)
	start = time.time()

	while time.time() - start < 3:
		msg.data = [4,topic_number]
		pub.publish(msg)

	listener()
