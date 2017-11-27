import cv2.cv as cv

import cv2
import numpy as np

from prepareimage import PrepareImage


class ObjectRecognizer:
    def __init__(self, image, cfgAccessor):
        self.image = image
        self.cfgAccessor = cfgAccessor

    def draw_circles(self, circles, image):
        for i in circles[0,:]:
            # draw the outer circle
            cv2.circle(image,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(image,(i[0],i[1]),2,(0,0,255),3)
        return image

    def find_circles(self, image=None):
        if (image is None):
            image = self.image
        circles = cv2.HoughCircles(image=image, method=cv.CV_HOUGH_GRADIENT,dp=1, minDist=20,param1=100, param2=30, minRadius=0, maxRadius=120)
        circles = np.uint16(np.around(circles))
        return circles

    def find_and_draw_circles(self, image=None):
        if (image is None):
            image = self.image
        color_image = cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
        imagePrarer = PrepareImage(image, self.cfgAccessor)
        #image = imagePrarer.threshold_image()
        circles = self.find_circles(image)
        return self.draw_circles(circles, color_image)