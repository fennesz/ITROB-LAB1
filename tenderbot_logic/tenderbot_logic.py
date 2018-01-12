#!/usr/bin/env python

import roshelper
import rospy
from statemachine import TenderBotStateMachine
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from camera_sensor.srv import *

node_name = "Logic"

n = roshelper.Node(node_name, anonymous=False)

# A class for TenderBot business logic
@n.entry_point()
class TenderBotLogic(object):
    implementation = None    
    
    @n.subscriber(rospy.get_namespace() + "Input/DrinkChoice", Int32)
    def drink_choice(self, choice):
        self.implementation.drinkchoice = choice.data;
    
    # ctor
    def __init__(self): # (self, exp_a, exp_b, exp_c)
        self.implementation = TenderBotStateMachine()
        self.implementation.start()

    @n.main_loop(frequency=30)
    def run(self):
        self.implementation.update()

if __name__ == "__main__":
    n.start(spin=True)
    
    
    
    
