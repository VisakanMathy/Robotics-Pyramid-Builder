#!/usr/bin/env python
import math
import time

import rospy
import rospkg


from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Int64,
    Int64MultiArray,
    MultiArrayDimension
)

from pick_place_functions import *


def callback(data):
    sys.exit(main(data.data))

def listener():
    rospy.init_node("left_arm_node")
    rospy.Subscriber('chatter',Int32,callback)
    rospy.spin()

def main(layer):
    topic_number = 0
    value = 0
    check = True
    count = 0
    brick = 2
    hover_distance = 0.1
    overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)
    
    rospy.on_shutdown(delete_gazebo_models)
    rospy.wait_for_message("/robot/sim/started", Empty)

    joint_angles = {'left_w0': 0.2182111448959421,
                    'left_w1': 1.806393354188649,
                    'left_w2': -0.5005068233099247,
                    'left_e0': -1.6671045472921753,
                    'left_e1': 1.3487800428036338,
                    'left_s0': 0.481109939761865,
                    'left_s1': -1.3757443780549539}
                             
    pnp = PickAndPlace('left', hover_distance)
    pnp.move_to_start(joint_angles)

    coordinate_list, pick_position = create_coordinates(layer)
    RArm, LArm = arrange_coordinates(coordinate_list,pick_position)

    block_poses = list()
              
    for coord_set in LArm:
        block_poses.append(Pose(position=Point(x=coord_set[0], y=coord_set[1], z=coord_set[2]),orientation=overhead_orientation))

    load_gazebo_models()
    load_brick_at_starting_point(0)

    while not rospy.is_shutdown() and count < len(block_poses):
        if check == False:
            pub = rospy.Publisher('move_right_arm_hub{}'.format(topic_number+1), Int64MultiArray, queue_size =10)
            msg = Int64MultiArray()
            msg.layout.dim = [MultiArrayDimension('',2,1)]
            if value == 1:
                pnp.move_to_start(joint_angles)
                brick+=2
                load_brick_at_starting_point(brick)
                msg.data = [2,topic_number]
                start = time.time()
                while time.time() - start < 2:
                    pub.publish(msg)
                topic_number+=1
            elif value == 2:
                pnp.place(block_poses[count])
                msg.data = [3,topic_number]
                count+=1
                start = time.time()
                while time.time()-start < 2:
                    pub.publish(msg)
                topic_number+=1
            elif value == 3:
                pnp.move_to_start(joint_angles)
                msg.data = [4,topic_number]
                start = time.time()
                while time.time()-start<2:
                    pub.publish(msg)
                topic_number +=1
            elif value == 4:
                pnp.pick(block_poses[count])
                msg.data = [1,topic_number]
                count+=1
                start = time.time()
                while time.time()-start<2:
                    pub.publish(msg)
                topic_number += 1
            check = True
        else:
            value = listen_to_hub(topic_number)
            check = False
    return 0

if __name__ == '__main__':
    listener()
