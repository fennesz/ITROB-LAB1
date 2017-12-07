#!/usr/bin/env python

import roshelper
import rospy
import os
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

    joint_states = {}

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

        #self.__setup_services()
        pass

    @n.subscriber(rospy.get_namespace() + "joint1/state", JointState)
    @n.subscriber(rospy.get_namespace() + "joint2/state", JointState)
    @n.subscriber(rospy.get_namespace() + "joint3/state", JointState)
    @n.subscriber(rospy.get_namespace() + "joint4/state", JointState)
    @n.subscriber(rospy.get_namespace() + "gripper/state", JointState)
    def update_joint_state(self, state):
        self.joint_states[state.name] = state
        
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
        return set_desired_posResponse(True)

    # function that handles requests to the angle service
    def handle_set_desired_angle(self, req):
        # Set desired effector pos
        self.desired_angle = req.angle
        return set_desired_angleResponse(True)

    @n.main_loop(frequency=30)
    def run(self):
        self.update_effector_info()
        self.publish_effector_pos()
        self.publish_effector_angle()
        self.publish_moving()



    # Example of IKine from teachers, figure out something else
    # I have no idea how it works
    def IKine(self, o, R):
        d1 = 10
        a1 = 5
        a2 = 20
        d4 = 20
        
        oc = o
        xc = oc[0]
        yc = oc[1]
        zc = oc[2]
        
        q1 = math.atan2(yc, xc)
        
        r2 = math.pow((xc - a1 * math.cos(q1)), 2) + math.pow((yc - a1 * math.sin(q1)), 2)
        s = zc - d1
        D = (r2 + math.pow(s, 2) - math.pow(a2, 2) - math.pow(d4, 2)) / (2 * a2 * d4)
        
        q3 = math.atan2(-math.sqrt(1 - math.pow(D, 2)), D)
        q2 = math.atan2(s, math.sqrt(r2)) - math.atan2(d4 * math.sin(q3), a2 + d4 * math.cos(q3))
        q4 = 0
        return [q1, q2, q3, q4]
        
if __name__ == "__main__":
    n.start(spin=True)
    
    
    
    
