#!/usr/bin/env python
import math
import time

import rospy
import rospkg
import numpy as np


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
from tf.transformations import *


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
    
    original_overhead = np.array([-0.0249590815779,
                             0.999649402929,
                             0.00737916180073,
                             0.00486450832011])
    place_overhead_orientation = Quaternion(
                             x=original_overhead[0],
                             y=original_overhead[1],
                             z=original_overhead[2],
                             w=original_overhead[3])
    
    second_overhead = quaternion_multiply(quaternion_from_euler(0,0,1.57),original_overhead)
    
    pick_overhead_orientation = Quaternion(
                         x=second_overhead[0],
                         y=second_overhead[1],
                         z=second_overhead[2],
                         w=second_overhead[3])


    rospy.on_shutdown(delete_gazebo_models)
    rospy.wait_for_message("/robot/sim/started", Empty)


    joint_angles = {'left_w0': -0.08042412041756966,
                    'left_w1': 1.6385748360237509,
                    'left_w2': -0.19069126257995908,
                    'left_e0': -0.490820123608967,
                    'left_e1': 1.6772691387232428,
                    'left_s0': -0.4379951625489787,
                    'left_s1': -1.776227964501175}

                             
    pnp = PickAndPlace('left', hover_distance)
    pnp.move_to_start(joint_angles)

    coordinate_list, pick_coordinates = create_coordinates_new(layer)
    RArm, LArm = arrange_coordinates(coordinate_list)

    block_poses = list()
              
    for coord_set in LArm:
        block_poses.append(Pose(position=Point(x=coord_set[0], y=coord_set[1], z=coord_set[2]),orientation=place_overhead_orientation))


    load_gazebo_models()
    load_orientated_brick(0)

    while not rospy.is_shutdown() and count < len(block_poses):
        if check == False:
            pub = rospy.Publisher('move_right_arm_hub{}'.format(topic_number+1), Int64MultiArray, queue_size =10)
            msg = Int64MultiArray()
            msg.layout.dim = [MultiArrayDimension('',2,1)]
            if value == 1:
                #pnp.move_to_start(joint_angles)
                pnp._approach(Pose(position=Point(x=0.6, y=0.35, z=0.2),orientation=place_overhead_orientation))
                brick+=2
                load_orientated_brick(brick)
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
                to_del = brick-4
                delete_brick(to_del)
                
            elif value == 3:
                #pnp.move_to_start(joint_angles)
                pnp._approach(Pose(position=Point(x=0.6, y=0.35, z=0.2),orientation=place_overhead_orientation))
                load_brick_in_structure(brick, LArm[count-1][0],LArm[count-1][1],LArm[count-1][2]+1)
                msg.data = [4,topic_number]
                start = time.time()
                while time.time()-start<2:
                    pub.publish(msg)
                topic_number +=1
            elif value == 4:
                pnp.pick(Pose(position=Point(x=pick_coordinates[0]-0.01, y=pick_coordinates[1], z=pick_coordinates[2]),orientation=pick_overhead_orientation))
                msg.data = [1,topic_number]
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
