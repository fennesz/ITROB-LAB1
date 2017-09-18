#!/usr/bin/env python

import rospy 
from std_msgs.msg    import Float64

class single_motor_node:
    def __init__(self):
        self.index = -1;
        self.cycle = [{'value': 0,   'period': 0.5},
                      {'value': 1,   'period': 1},
                      {'value': 0.8, 'period': 0.1},
                      {'value': 1,   'period': 0.1},
                      {'value': 0.8, 'period': 0.1},
                      {'value': 1,   'period': 0.1},
                      {'value': 0.8, 'period': 0.1},
                      {'value': 1,   'period': 0.1},
                      {'value': -1,  'period': 1},];
        self.joint_cmd_pub = rospy.Publisher("/joint1/command", Float64)
        #self.sub = rospy.Subscriber()
        self.timer = rospy.Timer(rospy.Duration(.5), self.on_timer)
        
    def on_timer(self,eventArg):
        print(self.index);
        rospy.loginfo("Timer called")
        self.timer.shutdown();
        self.index = self.index + 1;
        self.index = self.index if self.index < len(self.cycle) else 0;
        curEvent = self.cycle[self.index];
        self.joint_cmd_pub.publish(curEvent['value']);
        self.timer = rospy.Timer(rospy.Duration(curEvent['period']), self.on_timer);

if __name__ == "__main__":
    rospy.init_node("simple_motor_test")
    node = single_motor_node()
    rospy.spin()
