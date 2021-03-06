#!/usr/bin/env python

import roshelper
import rospy
from sensor_msgs.msg import Image
from camera_sensor.srv import *

node_name = "Vision"

n = roshelper.Node(node_name, anonymous=False)

# A class for the camera, requires image getting code
@n.entry_point()
class TenderBotVision(object):
    get_qr_codes = None
    get_qr_codes_service = None
    
    # Current raw image
    raw_image = None
    
    # Current shapes
    shapes = None
    
    # Current QRCodes
    qr_codes = ["0", "1"]
    
    # ctor, start service
    def __init__(self): # (self, exp_a, exp_b, exp_c)
        self.__setup_services()
        pass

    # Publishes the raw image
    @n.publisher(node_name + "/raw_image", Image)
    def publish_raw_image(self):
        msg = Image()
        # msg.data = self.raw_image
        return msg
    
    # Publishes the raw image
    @n.publisher(node_name + "/shapes", Image)
    def publish_shapes(self):
        msg = Image()
        # msg.data = self.shapes
        return msg

    # Private function that sets up the service handlers
    def __setup_services(self):
        self.get_qr_codes_service = rospy.Service(node_name + '/get_qr_codes',     # Name of service
                                                  get_qr_codes,                   # Service to implement
                                                  self.handle_get_qr_codes)       # Handler for service

    # function that handles requests to the position service
    def handle_get_qr_codes(self, req):
        return get_qr_codesResponse(self.qr_codes)

    @n.main_loop(frequency=30)
    def run(self):
        # self.update_image()
        # self.update_shapes()
        # self.update_qr_codes()
        self.publish_raw_image()
        self.publish_shapes()

if __name__ == "__main__":
    n.start(spin=True)
    
    
    
    
