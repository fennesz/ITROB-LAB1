#!/usr/bin/env python

import roshelper
import rospy
import os
import numpy as np
import math
from collections import deque
import time
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32, Float64, Bool
from dynamixel_msgs.msg import JointState

# For some reason it would not generate correct service files
# with the package being named "tenderbot_arm"
from TenderbotArm.srv import *




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
         [ 3.14 / 2,   0,     0,     3.14 / 2 ],
         [    0,       23.5,  0,     0        ],
         ]

    joint_states = {}

    raw_angles = [0, 0, 0, 0]
    
    corrected_angles = [0, 0, 0, 0]
    
    # correction variables
    angle_offsets = [-3.14 / 2, -3.14 / 2, 3.14 / 2, 0]
    #angle_offsets = [0,0,0,0]
    
    min_max_angles = 1.9
    
    # Baqse offset
    #base_offset = [36, 6, 0]
    base_offset = [0, 0, 0]

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
    
    # Gripper state
    desired_open = False
    gripper_open = False
    
    # Movingposition
    moving = False;
    
    # Services
    desired_pos_service = None
    desired_angle_service = None
    add_point_to_path_service = None
    
    # ctor, start service
    def __init__(self):
        cur_dir = os.path.dirname(os.path.realpath(__file__))
        self.__setup_services()

    @n.subscriber(rospy.get_namespace() + "joint1/state", JointState)
    @n.subscriber(rospy.get_namespace() + "joint2/state", JointState)
    @n.subscriber(rospy.get_namespace() + "joint3/state", JointState)
    @n.subscriber(rospy.get_namespace() + "joint4/state", JointState)
    @n.subscriber(rospy.get_namespace() + "gripper/state", JointState)
    def update_joint_state(self, state):
        self.joint_states[state.name] = state
        if(state.name.startswith("joint4")):
            self.effector_angle = state.current_pos
        elif(state.name.startswith("joint")):
            found_index = int(state.name[5]) - 1
            self.raw_angles[found_index] = state.current_pos;
            self.update_corrected_angle(found_index)
        elif(state.name == "gripper"):
            pass
    
    @n.publisher(rospy.get_namespace() + "joint1/command", Float64)
    def set_joint1_angle(self, angle):
        return max(min(angle, self.min_max_angles), -self.min_max_angles)
    
    @n.publisher(rospy.get_namespace() + "joint2/command", Float64)
    def set_joint2_angle(self, angle):
        return max(min(angle, self.min_max_angles), -self.min_max_angles)
    
    @n.publisher(rospy.get_namespace() + "joint3/command", Float64)
    def set_joint3_angle(self, angle):
        return max(min(angle, self.min_max_angles), -self.min_max_angles)
    
    @n.publisher(rospy.get_namespace() + "joint4/command", Float64)
    def set_joint4_angle(self, angle):
        return max(min(angle, self.min_max_angles), -self.min_max_angles)
    
    @n.publisher(rospy.get_namespace() + "gripper/command", Float64)
    def set_gripper_angle(self, angle):
        return max(min(angle, self.min_max_angles), -self.min_max_angles)
        
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
            
            parameters = self.dh
            for i in range(0, len(self.corrected_angles)): # Set the DH parameters to match current rotation
                parameters[i][0] = self.corrected_angles[i]
            # FKine here to find effector position
            pos_vector = self.fkine(parameters)
            self.effector_pos_x = pos_vector[0]
            self.effector_pos_y = pos_vector[1]
            self.effector_pos_z = pos_vector[2]

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
    
    @n.publisher(node_name + "/gripper_open", Bool)
    def publish_gripper_open(self):
        msg = Bool()
        msg.data = self.gripper_open
        return msg

    # Private function that sets up the service handlers
    def __setup_services(self):
        self.desired_pos_service = rospy.Service(node_name + '/set_desired_pos',         # Name of service
                                                 add_point,                              # Service to implement
                                                 self.handle_set_desired_pos)            # Handler for service
        self.add_point_to_path_service = rospy.Service(node_name + '/add_point_to_path', # Name of service
                                                 add_point,                              # Service to implement
                                                 self.add_point_to_path)                 # Handler for service
        self.desired_angle_service = rospy.Service(node_name + '/set_desired_angle',     # Name of service
                                                   set_desired_angle,                    # Service to implement
                                                   self.handle_set_desired_angle)        # Handler for service
        
    # function that handles requests to the position service
    def handle_set_desired_pos(self, req):
        if(not self.check_if_pos_possible([req.x, req.y, req.z, req.angle])):
            return add_pointResponse(False)
        self.path_points.clear() # Reset queue
        self.add_point_to_path(req)
        
        return add_pointResponse(True)

    def check_if_pos_possible(self, pos):
        return self.ikine(pos) != None

    def set_angles(self, angles):
        self.set_joint1_angle(angles[0])
        self.set_joint2_angle(angles[1])
        self.set_joint3_angle(angles[2])
        self.set_joint4_angle(angles[3])
        
    def close_gripper(self):
        self.set_gripper_angle(0)
        self.gripper_open = False
    
    def open_gripper(self):
        self.set_gripper_angle(1)
        self.gripper_open = True

    # function that handles requests to the angle service
    def handle_set_desired_angle(self, req):
        # Set desired effector pos
        self.desired_angle = req.angle
        self.set_joint4_angle(self.desired_angle)
        return set_desired_angleResponse(True)

    path_points = deque()
    
    def add_point_to_path(self, req):
        if(not self.check_if_pos_possible([req.x, req.y, req.z, req.angle])):
            return add_pointResponse(False)
        self.path_points.append([req.x, req.y, req.z, req.angle])
        return add_pointResponse(True)
    
    #Constants
    diffConst = 4 # Already squared
    rotConst = 0.2 # Also squared
    def move_along_path(self):
        if(self.moving):
            dist = (self.desired_pos_x - self.effector_pos_x)**2 + (self.desired_pos_y - self.effector_pos_y)**2 + (self.desired_pos_z - self.effector_pos_z)**2
            rot = abs(self.desired_angle - self.effector_angle)
            #rospy.loginfo(str(dist) + " " + str(rot))
            if(dist < self.diffConst and rot < self.rotConst):
                self.path_points.popleft()
                self.moving = False
        else:
            if(len(self.path_points) != 0):
                self.moving = True
                self.desired_pos_x = self.path_points[0][0]
                self.desired_pos_y = self.path_points[0][1]
                self.desired_pos_z = self.path_points[0][2]
                self.desired_angle = self.path_points[0][3]
                angles = self.ikine([self.desired_pos_x, self.desired_pos_y, self.desired_pos_z, self.desired_angle])
                self.set_angles(angles)

    @n.main_loop(frequency=30)
    def run(self):
        # update info
        self.update_effector_info()
        
        # Run work
        self.move_along_path()
        
        # Publish variables
        self.publish_effector_pos()
        self.publish_moving()
        self.publish_effector_angle()
        self.publish_gripper_open()

    def ikine(self, pos):
        # Robots base position
        Prx = 0
        Pry = self.dh[0][1]
        
        # Points position
        Px = math.sqrt(pos[0]**2 + pos[1]**2)
        Py = pos[2]
        
        # Robot lengths
        Len1 = self.dh[1][2]
        Len2 = self.dh[3][1]
        
        # Distance from first joint to point
        Dist = math.sqrt((Px - Prx)**2 + (Py - Pry)**2)
        
        if(Dist > (Len1 + Len2)):
            return None
        
        D1 = math.atan2(Py - Pry, Px - Prx)
        
        D2 = math.acos((Dist**2 + Len1**2 - Len2**2) / (2 * Dist * Len1)) # Law of Cosines
        
        # Base angle
        A1 = math.atan2(-pos[0], pos[1])
        
        # First joint angle
        A2 = D1 + D2
        
        A2 = A2 - (math.pi / 2) # Correction
        
        # Second joint angle
        A3 = math.acos((Len1**2 + Len2**2 - Dist**2) / (2 * Len1 * Len2)) # Law of Cosines
        A3 = A3 - math.pi
        
        if(abs(A1) > self.min_max_angles or abs(A2) > self.min_max_angles or abs(A3) > self.min_max_angles or abs(pos[3]) > self.min_max_angles):
            return None
        
        # I have no idea how to handle gripper rotation
        
        return [A1, A2, A3, pos[3]]
    
    # Forward kinematics
    def fkine(self, parameters):
        HomoMatrix = self.construct_homogeneous_matrix_from_dh(parameters[0][0], parameters[0][1], parameters[0][2], parameters[0][3])
        for i in range(1, len(self.dh)):
            new_matrix = self.construct_homogeneous_matrix_from_dh(parameters[i][0], parameters[i][1], parameters[i][2], parameters[i][3])
            HomoMatrix = np.dot(HomoMatrix, new_matrix)
        
        xyz = [HomoMatrix[0][3], HomoMatrix[1][3], HomoMatrix[2][3]]
        return xyz
        
    
    def construct_homogeneous_matrix_from_dh(self, theta, d, r, alpha):
        return [[math.cos(theta), -math.sin(theta) * math.cos(alpha),   math.sin(theta) * math.sin(alpha),  r * math.cos(theta)],
                [math.sin(theta), math.cos(theta) * math.cos(alpha),    -math.cos(theta) * math.sin(alpha), r * math.sin(theta)],
                [0,               math.sin(alpha),                      math.cos(alpha),                    d],
                [0, 0, 0, 1]]
        
if __name__ == "__main__":
    n.start(spin=True)
    
    
    
    
