#!/usr/bin/env python

import roshelper
import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32, Bool

nodeName = "Input" # Figure out how to set this from
                   # Arguments

n = roshelper.Node(nodeName, anonymous=False)

# A class for the manipulator arm, still needs actual servos, 
# motors and stuff
@n.entry_point() #(exp_a=1, exp_b=1, exp_c=1)
class Input(object):
    button_state = false
    last_button_state = false
    
    # ctor, start service
    def __init__(self): # (self, exp_a, exp_b, exp_c)
        pass
    
    def check_if_publish_button(self):
        return self.button_state == self.last_button_state
    
    # Publishes the end effector position
    @n.publisher(nodeName + "/button", Bool)
    def publish_button(self):
        msg = Bool();
        msg.data = self.button_state;
        return msg

    @n.main_loop(frequency=30)
    def run(self):
        if self.check_if_publish_button():
            self.publish_button()

if __name__ == "__main__":
    n.start(spin=True)
    