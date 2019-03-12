#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Inverse Kinematics Pick and Place Demo


###we changed the gripper size, and the slip coefficient, and slip value on model.sdf
"""
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
    Header,
    Empty,
    String,
    Int32,
    Int64,
    Int64MultiArray,
    MultiArrayDimension
)



import baxter_interface
from pick_place_functions import *

def callback(data):
    sys.exit(main(data.data))

def listener():
    rospy.init_node("right_arm_node")
    rospy.Subscriber('chatter',Int32,callback)
    rospy.spin()


def main(layer):
    value = 0
    topic_number = 0
    count = 0
    brick = 1
    hover_distance = 0.1
    check = True
    overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)


    rospy.on_shutdown(delete_gazebo_models)
    rospy.wait_for_message("/robot/sim/started", Empty)

    joint_angles = {'right_w0': 0.029706395093054415, 
                    'right_w1': 1.6307957600082124, 
                    'right_w2': 0.0712396684083455,                     
                    'right_e0': 0.09300972227969762, 
                    'right_e1': 1.6773273456786366,
                    'right_s0': 0.3110671146883546, 
                    'right_s1': -1.7486995749743792}
                         
    pnp = PickAndPlace('right', hover_distance)

    coordinate_list, pick_coordinates = create_coordinates(layer)
    RArm, LArm = arrange_coordinates(coordinate_list, pick_coordinates)
    pnp.move_to_start(joint_angles)

    block_poses = list()
    for coord_set in RArm:
        block_poses.append(Pose(position=Point(x=coord_set[0], y=coord_set[1], z=coord_set[2]),orientation=overhead_orientation))


    while not rospy.is_shutdown() and count < len(block_poses):
        if check == False:
            pub = rospy.Publisher('move_left_arm_hub{}'.format(topic_number+1), Int64MultiArray, queue_size =10)
            msg = Int64MultiArray()
            msg.layout.dim = [MultiArrayDimension('',2,1)]
            if value == 1:
                pnp.pick(block_poses[count])
                count+=1
                msg.data = [2,topic_number]
                start = time.time()
                while time.time()- start < 2:
                    pub.publish(msg)
                topic_number+=1
            elif value== 2:
                pnp.move_to_start(joint_angles)
                brick +=2
                load_brick_at_starting_point(brick)
                msg.data = [3,topic_number]
                start = time.time()
                while time.time()-start < 2:
                    pub.publish(msg)
                topic_number +=1
            elif value == 3:
                pnp.place(block_poses[count])
                count+=1
                msg.data = [4,topic_number]
                start = time.time()
                while time.time() - start < 2:
                    pub.publish(msg)
                topic_number +=1
            elif value == 4:
                pnp.move_to_start(joint_angles)
                msg.data = [1,topic_number]
                start = time.time()
                while time.time() - start < 2:
                    pub.publish(msg)
                topic_number +=1
            check = True
        else:
            value = listen_to_hub(topic_number)
            check = False

    return 0


if __name__ == '__main__':
    listener()
#Gareth Was Here
