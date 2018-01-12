#!/usr/bin/env python
import sys
sys.path.append('src/ITROB-LAB1/tenderbot_vision/')
from opencvlibrary.configaccess import ConfigAccessor
from opencvlibrary.cameracontroller import CameraController
import roshelper
from sensor_msgs.msg import Image as ImageMessage
from camera_sensor.srv import *

nodeName = "Vision" # Figure out how to set this from
                    # Arguments

n = roshelper.Node(nodeName, anonymous=False)

# A class for the camera, requires image getting code
@n.entry_point()
class TenderBotVision(object):
    get_qr_codes_service = None
    
    # ctor, start service
    def __init__(self): # (self, exp_a, exp_b, exp_c)
       # self.__setup_services()
        self.cfgAccess = ConfigAccessor(configName='tenderbot', location='catkin')
        self.cameraController = CameraController(self.cfgAccess)

    # Publishes the raw image
    @n.publisher(nodeName + "/raw_image", ImageMessage)
    def publish_raw_image(self):
        msg = ImageMessage()
        self.currentRawImage = self.cameraController.get_raw_webcam_image()
        msg.data = self.currentRawImage
        print 'returning image'
        return msg
    
    # Publishes the raw image
    @n.publisher(nodeName + "/shapes", ImageMessage)
    def publish_shapes(self):
        msg = ImageMessage()
        self.currentShapes = self.cameraController.get_shapes(self.currentRawImage)
        msg.data = self.currentShapes
        return msg

    @n.main_loop(frequency=5)
    def run(self):
        print "camera.py: Main loop executed"
        # self.update_image()
        # self.update_shapes()
        # self.update_qr_codes()
        self.publish_raw_image()
        self.publish_shapes()

if __name__ == "__main__":
    n.start(spin=True)
    
    
    
    
