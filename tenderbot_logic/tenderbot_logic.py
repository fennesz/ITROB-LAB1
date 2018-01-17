#!/usr/bin/env python

import roshelper
import rospy
from statemachine import TenderBotStateMachine
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3

from TenderbotArm.srv import add_point as add_point_srv

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy
node_name = "Logic"

n = roshelper.Node(node_name, anonymous=False)

# A class for TenderBot business logic
@n.entry_point()
class TenderBotLogic(object):
    state_machine = None
    drink_choice = None
    drinks = [
              {"ingredients": [1,1,1] },
              {"ingredients": [1,1,1] },
              {"ingredients": [1,1,1] },
              ]
    ingredient_pos = [
                      [25, 0, 5, 1],
                      [25, 10, 5, 1],
                      [25, 20, 5, 1]
                      ]
    current_ingredient = None
    
    shapes = None
    effector_pos = None
    
    @n.subscriber(rospy.get_namespace() + "Input/DrinkChoice", Int32)
    def drink_choice(self, choice):
        self.drinkchoice = choice.data;

    def fault_occurred(self):
        self.state_machine.error_occurred()
    
    def fault_cleared(self):
        self.state_machine.error_cleared()
    
    def force_arm_pos(self, pos):
        return pos
    
    @n.subscriber(rospy.get_namespace() + "Vision/shapes/red", Float32MultiArray)
    def get_shapes(self, shapes):
        self.shapes = shapes.data
    
    @n.subscriber(rospy.get_namespace() + "Arm/effector_pos", Vector3)
    def get_effector_pos(self, pos):
        self.effector_pos = pos

    def mix_drink(self):
        
        self.drink_choice = 0 # TESTESTESTETSTEST
        if (self.shapes is not None):
            rospy.loginfo(self.shapes)
            self.add_arm_point(self.shapes[0], self.shapes[1], 30, 0)  # Move over cup
        '''
        if(self.current_ingredient == None): # Start mixing
            #rospy.loginfo("Start mix")
            self.current_ingredient = 0
        if(self.current_ingredient >= len(self.drinks[self.drink_choice]["ingredients"])): # Done mixing
            #rospy.loginfo("Done mix")
            self.current_ingredient = None
            self.state_machine.drink_mixed()
        else: # Mixing
            #rospy.loginfo("Mix")
            pos = self.ingredient_pos[self.current_ingredient]
            self.add_arm_point(pos[0], pos[1], 30, 0)      # Move over cup
            self.add_arm_point(pos[0], pos[1], pos[2], pos[3])  # Move to cup
            self.current_ingredient = self.current_ingredient + 1
        '''
        
    
    # ctor
    def __init__(self): # (self, exp_a, exp_b, exp_c)
        add_point_path = rospy.get_namespace() + "Arm/add_point_to_path"
        rospy.wait_for_service(add_point_path)
        self.add_arm_point = rospy.ServiceProxy(add_point_path, add_point_srv)
        self.state_machine = TenderBotStateMachine(self)
        self.state_machine.start()

    @n.main_loop(frequency=30)
    def run(self):
        self.state_machine.update()

if __name__ == "__main__":
    n.start(spin=True)


    
    
    
