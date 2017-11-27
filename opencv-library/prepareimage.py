import cv2
import sys

import numpy as np

from configaccess import ConfigAccessor
from debugger import Debugger
from retrieveImage import RetrieveImage


class PrepareImage:

    def __init__(self, image, cfgAccessor): #Constructor
        self.cfgAccessor = cfgAccessor
        self.image = image
        self.reducedNoiseImage = self.reduce_picturenoise(self.image)
        self.imageGrey = image#self.greyscale(self.reducedNoiseImage)

    @staticmethod
    def greyscale(file):
        return cv2.cvtColor(file,cv2.COLOR_BGR2GRAY) #turn image to grayscale ---> turn back to colour later

    #reduce noise in picture
    def reduce_picturenoise(self, file):
        return cv2.medianBlur(file,5)

    #fill small holes and close "blobs"
    def threshold_image(self,debug=False):
        """
        Thresholds the image within the desired range and then dilates with a 3x3 matrix
        such that small holes are filled. Afterwards the 'blobs' are closed using a
        combination of dilate and erode
        """
        ret,th1 = cv2.threshold(self.imageGrey, self.cfgAccessor.data['thresholdValue'],255,cv2.THRESH_BINARY)
        if debug: cv2.imshow('th1',th1)
        resdi = cv2.dilate(th1,np.ones((3,3),np.uint8))
        if debug: cv2.imshow('dilated',resdi)
        closing = cv2.morphologyEx(resdi, cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
        if debug: cv2.imshow('closing',closing)
        return closing

    #Colour information is present within the specified range
    def extract_single_color_range(self,lower,upper):
        """
        Calculates a mask for which all pixels within the specified range is set to 1
        the ands this mask with the provided image such that color information is
        still present, but only for the specified range
        """
        self.hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(self.hsv, lower, upper)
        res = cv2.bitwise_and(self.image,self.image, mask= mask)
        return res


if __name__ == '__main__':
    retrImg = RetrieveImage()
    cfg = ConfigAccessor('tenderbot')
    fileInputString = sys.argv[1]
    fileInput = retrImg.get_from_file(fileInputString)
    prepareImg = PrepareImage(fileInput, cfg)
    reducedNoiseFile = prepareImg.reduce_picturenoise(fileInput)
    thresholdedImage = prepareImg.threshold_image()

    Debugger.show_image(thresholdedImage)
    #Debugger.show_image(prepareImg.imageGrey)

