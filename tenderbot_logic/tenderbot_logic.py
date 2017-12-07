#!/usr/bin/env python

import roshelper
import rospy
from statemachine import TenderBotStateMachine
from sensor_msgs.msg import Image
from camera_sensor.srv import *

node_name = "Logic"

n = roshelper.Node(node_name, anonymous=False)

# A class for the camera, requires image getting code
@n.entry_point()
class TenderBotLogic(object):
    
    implementation = None
    
    # ctor, start service
    def __init__(self): # (self, exp_a, exp_b, exp_c)
        self.implementation = TenderBotStateMachine()
        self.implementation.start()
        pass

    @n.main_loop(frequency=30)
    def run(self):
        self.implementation.update()
        pass

if __name__ == "__main__":
    n.start(spin=True)
    
    
    
    
