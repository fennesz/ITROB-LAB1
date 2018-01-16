#!/usr/bin/env python
import sys
sys.path.append('src/ITROB-LAB1/tenderbot_vision/')
from opencvlibrary.configaccess import ConfigAccessor
from opencvlibrary.cameracontroller import CameraController
import roshelper
from sensor_msgs.msg import Image as ImageMessage
#from tenderbot_vision.msg import ShapeMessage
from camera_sensor.srv import *
import rospy

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
        self.currentRawImage = self.cameraController.get_raw_webcam_image(fromFile=False)
        msg.data = self.currentRawImage
        print 'returning image'
        return msg
    
    # Publishes the shapes
    '''@n.publisher(nodeName + "/shapes", ShapeMessage)
    def publish_shapes(self):
        msg = ShapeMessage()
        redShapes = self.cameraController.get_shapes(self.currentRawImage, color='red')
        for i,redShape in enumerate(redShapes):
            msg.data.x[i] = redShape[0]
            msg.data.y[i] = redShape[1]
            msg.data.radius[i] = redShape[2]
            msg.data.color = 'red'
        rospy.loginfo(msg.data)
        return msg
        '''

    def publish_shapes(self):
        self.publish_shapes_all()
        self.publish_shapes_red()
        self.publish_shapes_yellow()
        self.publish_shapes_green()
        self.publish_shapes_blue()

    # Publishes the shapes
    @n.publisher(nodeName + "/shapes/all", ImageMessage)
    def publish_shapes_all(self):
        msg = ImageMessage()
        msg.data = self.cameraController.get_shapes(self.currentRawImage, color='all', show=False)
        return msg

    # Publishes the shapes red
    @n.publisher(nodeName + "/shapes/red", ImageMessage)
    def publish_shapes_red(self):
        msg = ImageMessage()
        msg.data = self.cameraController.get_shapes(self.currentRawImage, color='red', show=True)
        return msg

    # Publishes the shapes green
    @n.publisher(nodeName + "/shapes/green", ImageMessage)
    def publish_shapes_green(self):
        msg = ImageMessage()
        msg.data = self.cameraController.get_shapes(self.currentRawImage, color='green', show=False)
        return msg


    # Publishes the shapes yellow
    @n.publisher(nodeName + "/shapes/yellow", ImageMessage)
    def publish_shapes_yellow(self):
        msg = ImageMessage()
        msg.data = self.cameraController.get_shapes(self.currentRawImage, color='yellow', show=False)
        return msg

    # Publishes the shapes blue
    @n.publisher(nodeName + "/shapes/blue", ImageMessage)
    def publish_shapes_blue(self):
        msg = ImageMessage()
        msg.data = self.cameraController.get_shapes(self.currentRawImage, color='blue', show=False)
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
    
    
    

