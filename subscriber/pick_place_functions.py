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
import argparse
import struct
import sys
import copy
import numpy as np
import math
import time

import rospy
import rospkg


from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
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

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface

class PickAndPlace(object):
    def __init__(self, limb, hover_distance = 0.2, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        #self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - ")
                    #Valid Joint Solution Found from Seed Type: {0}".format(
                     #    (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution: \n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def print_position(self):
        current_pose = self._limb.endpoint_pose()
        pprint(current_pose)
        x = current_pose['position'].x
        y = current_pose['position'].y
        z = current_pose['position'].z
        print(x, y, z)

    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    def pick(self, pose):
        self._hover_distance = self._hover_distance/2
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()
        self._hover_distance = self._hover_distance*2

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

        
def load_gazebo_models(table_pose=Pose(position=Point(x=0.82, y=0, z=-0.05)),
                       table_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

        
def load_brick_at_starting_point(brick_number, brick_pose=Pose(position=Point(x=0.49, y=0.01, z=0.752)),
                brick_reference_frame =  'world'):

    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    # Load Table SDF
    
    brick_xml = ''

    with open (model_path + "new_brick/model.sdf", "r") as brick_file:
        brick_xml=brick_file.read().replace('\n', '')

    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("new_brick_left{}".format(brick_number), brick_xml, "/",
                             brick_pose, brick_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
        # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    



def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))


def delete_bricks(brick):
    for item in range(brick):
        try:
            resp_delete = delete_model('new_brick{}'.format(item))
        except rospy.ServiceException, e:
            rospy.loginfo("Delete Model service call failed: {0}".format(e))
     
def listen_to_hub(topic_number):
    print("listening to hub for {}".format(topic_number))
    msg = rospy.wait_for_message('do_shit{}'.format(topic_number),Int64MultiArray)
    value = msg.data[0]
    return value

def create_coordinates(layer):
    sc = 1; #this scales all the brick dimensions and the gaps between the bricks
    #brick dimensions
    xbrick = 0.192; #this is the length of the brick
    ybrick = 0.086; #width of brick
    zbrick = 0.062; #height of brick
    #Scaled brick dimensions
    xb = xbrick*sc
    yb = ybrick*sc
    zb = zbrick*sc
    #starting position`
    xs = 0.49; #this is the starting position along length
    ys = 0.01; #starting position along height
    zs = 0.11; #starting position along width

    s = [xs, ys, zs] #this is the permanent starting position which is reeferenced as the centre of the brick

    #Input for number of layers
    x = layer
    print(layer)
    x1 = list(range(1,x+1))
    x1.reverse()
    layers = np.asarray(x1)
    m = layers.tolist() #this is a list of number of bricks in each layer. e.g. [3, 2, 1] is 3 bricks in base layer, 2 in second etc

    lay = [] #this is the matrix containint all the coordinates for the pyramid in x, y, z which corrrespond to length, depth, width with the brick normally orientated
    x_structure = round(xs + xb + sc*0.05,4)

    for j in range(x):
        z_structure = zs + (x-j-1)*(zb) + 0.03
        
        if (j+1)%2 == 0: #even layers
            init_list = list(range(int((j+1)/2)))
            rev_list = list(range(int((j+1)/2)))

            for i in range(len(init_list)):
                init_list[i] = i+0.5
                rev_list[i] = -(i+0.5) 
            
            rev_list.reverse()   
            total_list = rev_list+init_list

        elif (j+1)%2 == 1: #odd layers
            pos_list = list(range(int((j+2)/2)))
            neg_list = list(range(int(-((j)/2)),0))
            total_list = neg_list + pos_list
            
        for item in total_list:
            y_structure = ys + item*(yb + 0.03)
                
            c = [round(x_structure,4), round(y_structure,4), round(z_structure,4)]
            
            cnew = [] #Creates a new matrix
            cnew = c #Makes cnew equal to the c coordinates
            lay.append(cnew)
            
      
    lay.reverse()  
    print('lay', lay)
    return lay, s

def arrange_coordinates(lay, s):
    RArm = []
    LArm = []

    while len(lay) != 0:
        z_smallest = lay[0][2]
        list_of_index = []
        for i in range(len(lay)):
            if lay[i][2] < z_smallest:
                list_of_index = [i]
            elif lay[i][2] == z_smallest:
                list_of_index.append(i)
                   
        y_largest_index = list_of_index[0]
        
        for item in list_of_index:
            if lay[item][1] > lay[y_largest_index][1]:
                y_largest_index = item
        LArm.append(s)
        LArm.append(lay[y_largest_index])
        lay.remove(lay[y_largest_index])    
              
        if len(lay) != 0:
            z_smallest = lay[0][2]
            list_of_index = []
            for i in range(len(lay)):
                if lay[i][2] < z_smallest:
                    list_of_index = [i]
                elif lay[i][2] == z_smallest:
                    list_of_index.append(i)
        
            y_smallest_index = list_of_index[0]
            for item in list_of_index:
                if lay[item][1] < lay[y_smallest_index][1]:
                    y_smallest_index = item
            RArm.append(s)
            RArm.append(lay[y_smallest_index])
            lay.remove(lay[y_smallest_index])  
    return RArm, LArm    

                
                    
