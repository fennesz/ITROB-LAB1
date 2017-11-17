import cv2.cv as cv

import cv2
import numpy as np


class ObjectRecognizer:
    def __init__(self, image):
        self.image = image

    def draw_circles(self, circles, image):
        for i in circles[0,:]:
            #draw the outer circle
            cv2.circle(image,(i[0],i[1],i[2],(0,255,0),2))
            #draw the center of the circle
            cv2.circle(image,(i[0],i[1]),2,(0,0,255),3)

    def find_circles(self, image=None):
        if (image is None):
            image = self.image
        width = image.shape[:2]
        circles = cv2.HoughCircles(image=image, method=cv.CV_HOUGH_GRADIENT,dp=1, minDist=20,param1=50, param2=30, minRadius=0, maxRadius=120)
        circles = np.uint16(np.around(circles))
        return circles

    def find_and_draw_circles(self, image=None):
        if (image is None):
            image = self.image
        circles = self.find_circles(image)
        return self.draw_circles(circles, image)