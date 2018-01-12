import cv2
import sys

import numpy as np

from configaccess import ConfigAccessor
from debugger import Debugger
from retrieveImage import RetrieveImage

# minDist=self.cfgAccessor.data['minDistance']


class PrepareImage:
    # BLUE
    #lower_blue = np.array([100, 50, 50])
    #upper_blue = np.array([130, 255, 255])

    # GREEN
    #lower_green = np.array([35, 50, 50])
    #upper_green = np.array([90, 255, 255])

    # YELLOW
    #lower_yellow = np.array([20, 50, 50])
    #upper_yellow = np.array([30, 255, 255])

    # RED
    #lower_red = np.array([0, 50, 50])
    #upper_red = np.array([20, 255, 255])

    def __init__(self, image, cfgAccessor): #Constructor
        self.cfgAccessor = cfgAccessor
        self.image = image
        self.reducedNoiseImage = self.reduce_picturenoise(self.image)

    def greyscale(self, image=None):
        if image is None:
            image = self.image
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY, 0) #turn image to grayscale ---> turn back to colour later

    #reduce noise in picture
    def reduce_picturenoise(self, file):
        return cv2.medianBlur(file,5)

    #fill small holes and close "blobs"
    def threshold_image(self, debug=False, image=None):
        """
        Thresholds the image within the desired range and then dilates with a 3x3 matrix
        such that small holes are filled. Afterwards the 'blobs' are closed using a
        combination of dilate and erode
        """        
        if (image is None):
            image = self.image
        ret,th1 = cv2.threshold(image, self.cfgAccessor.data['thresholdValue'],255,cv2.THRESH_BINARY)
        #TODO: Read about two commands below:
        resdi = cv2.dilate(th1,np.ones((self.cfgAccessor.data['dilatePixelsX'],self.cfgAccessor.data['dilatePixelsY']),np.uint8))
        closing = cv2.morphologyEx(resdi, cv2.MORPH_CLOSE,np.ones((self.cfgAccessor.data['closePixelsX'],self.cfgAccessor.data['closePixelsY']),np.uint8))
        return closing

    #Colour information is present within the specified range
    def extract_single_color_range(self,lower=None,upper=None, color=None):
        if color is not None:
            upper, lower = self.__get_color(color)
        self.hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(self.hsv, lower, upper)
        self.singleColorImage = cv2.bitwise_and(self.image,self.image, mask=mask)
        return self.singleColorImage

    def __get_color(self, color):
        self.lower_blue = np.array([self.cfgAccessor.data['lowBlue1'], 50, 50])
        self.upper_blue = np.array([self.cfgAccessor.data['upBlue1'], 255, 255])
        self.lower_green = np.array([self.cfgAccessor.data['lowGreen1'], 50, 50])
        self.upper_green = np.array([self.cfgAccessor.data['upGreen1'], 255, 255])
        self.lower_yellow = np.array([self.cfgAccessor.data['lowYel1'], 50, 50])
        self.upper_yellow = np.array([self.cfgAccessor.data['upYel1'], 255, 255])
        self.lower_red = np.array([self.cfgAccessor.data['lowRed1'], 50, 50])
        self.upper_red = np.array([self.cfgAccessor.data['upRed1'], 255, 255])        
        if color == 'red':
            return self.upper_red, self.lower_red
        if color == 'blue':
            return self.upper_blue, self.lower_blue
        if color == 'green':
            return self.upper_green, self.lower_green
        if color == 'yellow':
            return self.upper_yellow, self.lower_yellow
        if color == 'all':
            return np.array([255, 255, 255]), np.array([0,0,0])
        raise Exception("Invalid color. Choose red, green, yellow or blue")


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

