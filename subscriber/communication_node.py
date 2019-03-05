#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32, String, Int64MultiArray, MultiArrayDimension, Int64
import time
global move_left
global move_right
global topic_number 


topic_number = 0
move_left = [6,8]
move_right = [5,7]

def callbackleft(data):
	global move_left 
	move_left = data.data

def callbackright(data):
	global move_right
	move_right = data.data

def callback(data):
	global topic_number
	global move_left
	global move_right
	pub = rospy.Publisher('do_shit{}'.format(topic_number), Int64MultiArray, queue_size = 10)
	msg = Int64MultiArray()
	msg.layout.dim = [MultiArrayDimension('',2,1)]
	r = rospy.Rate(10)
	start = time.time()
	print('here')

	while time.time()-start < 5:
		#print(data)
		print(msg.data)
		#print('hi')
		if data == 1:
			msg.data = [1,topic_number]
			pub.publish(msg)  #left picked right moving to pick
		elif data== 2:
			msg.data = [2,topic_number]
			pub.publish(msg)	#right picked left moving to place
		elif data== 3:
			msg.data = [3,topic_number]
			pub.publish(msg)	#left placed right moving to placed
		elif data == 4:
			msg.data = [4,topic_number]
			pub.publish(msg)	#right placed left moving to pick
			#print('here')
	move_right = [5,7]
	move_left = [6,8]
		
		



def listener():
	global move_left
	global move_right
	global topic_number
	while not rospy.is_shutdown():
		#print("listening")
		'''
		rospy.Subscriber('move_left_arm_hub{}'.format(topic_number+1), Int64MultiArray, callbackleft)
		rospy.Subscriber('move_right_arm_hub{}'.format(topic_number+1), Int64MultiArray, callbackright)
		'''
		if topic_number%2 == 0:
			msg_left = rospy.wait_for_message('move_left_arm_hub{}'.format(topic_number+1), Int64MultiArray)
			msg_right= rospy.wait_for_message('move_right_arm_hub{}'.format(topic_number+1), Int64MultiArray)
		else:
			msg_right = rospy.wait_for_message('move_right_arm_hub{}'.format(topic_number+1), Int64MultiArray)
			msg_left =  rospy.wait_for_message('move_left_arm_hub{}'.format(topic_number+1),Int64MultiArray)


		move_left = msg_left.data
		move_right =msg_right.data

		#print('topic: {} \tleft: {}\t right: {}'.format(topic_number,move_left,move_right))

		#print('move right {}'.format(move_right))
		#print('move left {}'.format(move_left))

		if move_left[1] == topic_number and move_right[1] == topic_number:
			topic_number +=1
			print('move right {}'.format(move_right))
			print('move left {}'. format(move_left))
			callback(move_left[0])
			
	#rospy.spin()



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
