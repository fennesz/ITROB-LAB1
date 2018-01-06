import cv2.cv as cv

import cv2
import numpy as np


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
        circles = cv2.HoughCircles(image=image,
                                   method=cv.CV_HOUGH_GRADIENT,
                                   dp=self.cfgAccessor.data['dp'],
                                   minDist=self.cfgAccessor.data['minDistance'],
                                   param1=self.cfgAccessor.data['param1'],
                                   param2=self.cfgAccessor.data['param2'],
                                   minRadius=self.cfgAccessor.data['minRadius'],
                                   maxRadius=self.cfgAccessor.data['maxRadius'])
        if circles is not None:
            circles = np.uint16(np.around(circles))
        return circles

    def find_and_draw_circles(self, image=None):
        if (image is None):
            image = self.image
        color_image = cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
        circles = self.find_circles(image)
        if circles is not None:
            return self.draw_circles(circles, color_image)
        return color_image