#!/usr/bin/env python

import roshelper
import rospy
import os
import numpy as np
import math
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32, Float64, Bool
from dynamixel_msgs.msg import JointState
from manipulator_arm.srv import *


node_name = "Arm"
n = roshelper.Node(node_name, anonymous=False)

# A class for the manipulator arm, still needs actual servos, 
# motors and stuff
@n.entry_point() #(exp_a=1, exp_b=1, exp_c=1)
class TenderBotArm(object):

    # DH Parameters
    #        Theta      D       R     Alpha
    dh = [
         [ 3.14 / 2,   15.22, 0,    -3.14 / 2 ],
         [-3.14 / 2,   0,     17.2,  0        ],
         [ 3.14 / 2,   0,     2.5,   3.14 / 2 ],
         [    0,       23.5,  0,     0        ],
         ]

    joint_states = {}

    raw_angles = [0, 0, 0, 0]
    
    corrected_angles = [0, 0, 0, 0]
    
    # correction variables
    angle_offsets =    [-3.14 / 2, -3.14 / 2, 3.14 / 2, 0]
    
    # Base offset
    base_offset = [36, 6, 0]

    # 3d vector of the effectors position
    effector_pos_x = 0.0
    effector_pos_y = 0.0
    effector_pos_z = 0.0
    
    # 3d vector of desired location
    desired_pos_x = 0.0
    desired_pos_y = 0.0
    desired_pos_z = 0.0
    
    # Angle of end effector
    effector_angle = 0.0
    
    # Desired angle of end effector
    desired_angle = 0.0
    
    # Movingposition
    moving = False;
    
    # Services
    desired_pos_service = None
    desired_angle_service = None
    
    # ctor, start service
    def __init__(self):
        cur_dir = os.path.dirname(os.path.realpath(__file__))

        self.__setup_services()
        pass

    @n.subscriber(rospy.get_namespace() + "joint1/state", JointState)
    @n.subscriber(rospy.get_namespace() + "joint2/state", JointState)
    @n.subscriber(rospy.get_namespace() + "joint3/state", JointState)
    @n.subscriber(rospy.get_namespace() + "joint4/state", JointState)
    @n.subscriber(rospy.get_namespace() + "gripper/state", JointState)
    def update_joint_state(self, state):
        self.joint_states[state.name] = state
        if(state.name.startswith("joint")):
            found_index = int(state.name[5]) - 1
            self.raw_angles[found_index] = state.current_pos;
            self.update_corrected_angle(found_index)
    
    @n.publisher(rospy.get_namespace() + "joint1/command", Float64)
    def set_joint1_angle(self, angle):
        return max(min(angle, 1.57), -1.57)
    
    @n.publisher(rospy.get_namespace() + "joint2/command", Float64)
    def set_joint2_angle(self, angle):
        return max(min(angle, 1.57), -1.57)
    
    @n.publisher(rospy.get_namespace() + "joint3/command", Float64)
    def set_joint3_angle(self, angle):
        return max(min(angle, 1.57), -1.57)
    
    @n.publisher(rospy.get_namespace() + "joint4/command", Float64)
    def set_joint4_angle(self, angle):
        return max(min(angle, 1.57), -1.57)
        
    
    def update_corrected_angle(self, index):
        self.corrected_angles[index] = self.correct_angle(self.raw_angles[index], index)
    
    def correct_angle(self, angle, index):
        return angle + self.angle_offsets[index]
    
    def update_effector_info(self):
        rotations = []
        if(len(self.joint_states.items()) == 5):
            rotations.append(self.joint_states["joint1"].current_pos)
            rotations.append(self.joint_states["joint2"].current_pos)
            rotations.append(self.joint_states["joint3"].current_pos)
            rotations.append(self.joint_states["joint4"].current_pos)
            # FKine here to find effector position
            # pos_vector = FKine()
            # self.effector_pos_x = 0
            # self.effector_pos_x = 0
            # self.effector_pos_x = 0
            # Also update rotation of end effector
            # self.effector_angle = 0

    # Publishes the end effector position
    @n.publisher(node_name + "/effector_pos", Vector3)
    def publish_effector_pos(self):
        msg = Vector3();
        msg.x = self.effector_pos_x
        msg.y = self.effector_pos_y
        msg.z = self.effector_pos_z
        return msg
    
    # Publishes the end effector angle
    @n.publisher(node_name + "/effector_angle", Float32)
    def publish_effector_angle(self):
        msg = Float32()
        msg.data = self.effector_angle
        return msg
    
    # Publishes whether or not the arm is moving
    @n.publisher(node_name + "/moving", Bool)
    def publish_moving(self):
        msg = Bool()
        msg.data = self.moving
        return msg

    # Private function that sets up the service handlers
    def __setup_services(self):
        self.desired_pos_service = rospy.Service(node_name + '/set_desired_pos',     # Name of service
                                                 set_desired_pos,                   # Service to implement
                                                 self.handle_set_desired_pos)       # Handler for service
        self.desired_angle_service = rospy.Service(node_name + '/set_desired_angle', # Name of service
                                                   set_desired_angle,               # Service to implement
                                                   self.handle_set_desired_angle)   # Handler for service
        
    # function that handles requests to the position service
    def handle_set_desired_pos(self, req):
        # Set desired effector pos
        self.desired_pos_x = req.x
        self.desired_pos_y = req.y
        self.desired_pos_z = req.z
        
        #Test
        angles = self.ikine([-req.y, req.x, req.z])
        rospy.loginfo(angles)
        self.set_joint1_angle(angles[0])
        self.set_joint2_angle(angles[1])
        self.set_joint3_angle(angles[2])
        self.set_joint4_angle(angles[3])
        
        return set_desired_posResponse(True)

    # function that handles requests to the angle service
    def handle_set_desired_angle(self, req):
        # Set desired effector pos
        self.desired_angle = req.angle
        return set_desired_angleResponse(True)

    @n.main_loop(frequency=30)
    def run(self):
        pass





    # Example of IKine from teachers, figure out something else
    # I have no idea how it works
    def ikine(self, pos_coords):
        
        test = "\n"
        
        # Get DH parameters
        d1 = self.dh[0][1]
        a1 = self.dh[0][2]
        a2 = self.dh[1][2]
        d4 = self.dh[3][1]
        
        test = test + "DH: d1 " + str(d1) + " a1 " + str(a1) + " a2 " + str(a2) + " d4 " + str(d4) + "\n"
        
        xc = pos_coords[0]
        yc = pos_coords[1]
        zc = pos_coords[2]
        
        test = test + "XYZ: " + str(xc) + ", " + str(yc) + ", "+ str(zc) + "\n"
        
        # Base rotation
        q1 = math.atan2(yc, xc)
        
        test = test + "q1: " + str(q1) + "\n"
        
        # Radius squared
        r2 = math.pow((xc - a1 * math.cos(q1)), 2) + math.pow((yc - a1 * math.sin(q1)), 2)
        
        test = test + "r2: " + str(r2) + "\n"
        
        s = zc - d1
        
        test = test + "s: " + str(s) + "\n"
        
        D = (r2 + math.pow(s, 2) - math.pow(a2, 2) - math.pow(d4, 2)) / (2 * a2 * d4)
        
        test = test + "D: " + str(D) + "\n"
        
        # Third theta
        q3 = math.atan2(-math.sqrt(1 - math.pow(D, 2)), D)
        
        # Second theta
        q2 = math.atan2(s, math.sqrt(r2)) - math.atan2(d4 * math.sin(q3), a2 + d4 * math.cos(q3))
        
        # Last theta
        q4 = 0
        
        angles = [q1, q2, q3, q4]
        
        #rospy.loginfo([angles])
        
        return angles
    
    # Forward kinematics
    def fkine(self, parameters = None):
        if(parameters == None):
            parameters = self.dh
            for i in range(0, len(self.corrected_angles)): # Set the DH parameters to match current rotation
                parameters[i][0] = self.raw_angles[i]
                 
        HomoMatrix = self.construct_homogeneous_matrix_from_dh(parameters[0][0], parameters[0][1], parameters[0][2], parameters[0][3])
        for i in range(1, len(self.dh)):
            new_matrix = self.construct_homogeneous_matrix_from_dh(parameters[i][0], parameters[i][1], parameters[i][2], parameters[i][3])
            HomoMatrix = np.dot(HomoMatrix, new_matrix)
        
        xyz = [HomoMatrix[0][3], HomoMatrix[1][3], HomoMatrix[2][3]]
        xyz[0] = xyz[0] + self.base_offset[0]
        xyz[1] = xyz[1] + self.base_offset[1]
        xyz[2] = xyz[2] + self.base_offset[2]
        return xyz
        
    
    def construct_homogeneous_matrix_from_dh(self, theta, d, r, alpha):
        return [[math.cos(theta), -math.sin(theta) * math.cos(alpha),   math.sin(theta) * math.sin(alpha),  r * math.cos(theta)],
                [math.sin(theta), math.cos(theta) * math.cos(alpha),    -math.cos(theta) * math.sin(alpha), r * math.sin(theta)],
                [0,               math.sin(alpha),                      math.cos(alpha),                    d],
                [0, 0, 0, 1]]
        
if __name__ == "__main__":
    n.start(spin=True)
    
    
    
    
