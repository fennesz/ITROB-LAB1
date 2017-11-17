#!/usr/bin/env python

import roshelper
import rospy
from geometry_msgs.msg import Vector3
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
    
    # Moving
    moving = false;
    
    desired_post_service = None
    
    # ctor, start service
    def __init__(self): # (self, exp_a, exp_b, exp_c)
        self.__setup_desired_pos_service()
        pass

    # Publishes the end effector position
    @n.publisher(nodeName + "/effector_pos", Vector3)
    def publish_effector_pos(self):
        pos = Vector3();
        pos.x = self.effector_pos_x
        pos.y = self.effector_pos_y
        pos.z = self.effector_pos_z
        return pos
    
    # Publishes whether or not the arm is moving
    @n.publisher(nodeName + "/moving", bool)
    def public_moving(self):
        return self.moving

    # Private function that sets up the service handler
    def __setup_desired_pos_service(self):
        self.desired_post_service = rospy.Service( nodeName + '/set_desired_pos', # Name of service
                                             set_desired_pos,                # Service to implement
                                             self.handle_set_desired_pos)        # Handler for service
        
    # function that handles requests to the service
    def handle_set_desired_pos(self, req):
        # Set desired effector pos
        self.desired_pos_x = req.x
        self.desired_pos_y = req.y
        self.desired_pos_z = req.z
        return set_desired_posResponse(True)

    @n.main_loop(frequency=30)
    def run(self):
        self.publish_effector_pos()
        self.public_moving()

if __name__ == "__main__":
    n.start(spin=True)
    
    
    
    
