#!/usr/bin/env python

import roshelper
import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32, Bool
from manipulator_arm.srv import *

nodeName = "Manipulator" # Figure out how to set this from
                         # Arguments
n = roshelper.Node(nodeName, anonymous=False)

# A class for the manipulator arm, still needs actual servos, 
# motors and stuff
@n.entry_point() #(exp_a=1, exp_b=1, exp_c=1)
class Manipulator(object):

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
    
    # Moving
    moving = False;
    
    desired_pos_service = None
    desired_angle_service = None
    
    # ctor, start service
    def __init__(self): # (self, exp_a, exp_b, exp_c)
        self.__setup_services()
        pass

    # Publishes the end effector position
    @n.publisher(nodeName + "/effector_pos", Vector3)
    def publish_effector_pos(self):
        msg = Vector3();
        msg.x = self.effector_pos_x
        msg.y = self.effector_pos_y
        msg.z = self.effector_pos_z
        return msg
    
    # Publishes the end effector angle
    @n.publisher(nodeName + "/effector_angle", Float32)
    def publish_effector_angle(self):
        msg = Float32()
        msg.data = self.effector_angle
        return msg
    
    # Publishes whether or not the arm is moving
    @n.publisher(nodeName + "/moving", Bool)
    def public_moving(self):
        msg = Bool()
        msg.data = self.moving
        return msg

    # Private function that sets up the service handlers
    def __setup_services(self):
        self.desired_pos_service = rospy.Service(nodeName + '/set_desired_pos',     # Name of service
                                                 set_desired_pos,                   # Service to implement
                                                 self.handle_set_desired_pos)       # Handler for service
        self.desired_angle_service = rospy.Service(nodeName + '/set_desired_angle', # Name of service
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
        self.publish_effector_pos()
        self.publish_effector_angle()
        self.public_moving()

if __name__ == "__main__":
    n.start(spin=True)
    
    
    
    
