#!/usr/bin/env python
import cv2
import numpy as np
import sys
import urllib
import errno
import os
from twisted.protocols.ftp import FileNotFoundError
import datetime
import math

#Code inspired from opencvexample.py and opencv Hough Cicle Transform website (https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutotials/py_imgproc/py_houghcircles/py_houghcircles.html)

#get image from file ---> code from retrieveImage.py

def get_from_file(filename):
	"""
	Loads image from file
	"""
	print "Loading from file..."
	file = cv2.imread(filename)
	if file is not None:
		print "Succesfully loaded file"
		return file
	else:
		print "Couldn't find file: " + filename
		raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), filename)
	
	file = cv2.medianBlur(file,5) #reduce noise in picture

#Colour information is present within the specified range
def extract_single_color_range(image,hsv,lower,upper):
    """
    Calculates a mask for which all pixels within the specified range is set to 1
    the ands this mask with the provided image such that color information is
    still present, but only for the specified range
    """
    mask = cv2.inRange(hsv, lower, upper)
    res = cv2.bitwise_and(image,image, mask= mask)
    return res

#fill small holes and close "blobs"
def threshold_image(image,debug=False):
    """
    Thresholds the image within the desired range and then dilates with a 3x3 matrix
    such that small holes are filled. Afterwards the 'blobs' are closed using a
    combination of dilate and erode
    """
    ret,th1 = cv2.threshold(image,50,255,cv2.THRESH_BINARY)
    if debug: cv2.imshow('th1',th1)
    resdi = cv2.dilate(th1,np.ones((3,3),np.uint8))
    if debug: cv2.imshow('dilated',resdi)
    closing = cv2.morphologyEx(resdi, cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
    if debug: cv2.imshow('closing',closing)

    return closing

def show_circles(image,debug=False):
	grey_file = cv2.cvtColor(file,cv2.COLOR_BGR2GRAY) #turn image to grayscale ---> turn back to colour later

	#locate circles in the picture
	circles = cv2.HoughCircles(file, cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=0,maxRadius=0)
	circles = np.uint16(np.around(circles))
	
	return show_circles

#process image and detect circles in the given hsv color range
def do_full(image,hsv,upper,lower,debug=False):
	
	single_color_img = extract_single_color_range(image,hsv,lower,upper)

	if debug:
		cv2.imshow('single_color_image', single_color_img)
	single_channel = threshold_image(single_color_img,debug)
	
	if debug:
		cv2.imshow('single_channel', single_channel)
	cont,hierarchy = show_circles(single_channel,debug)

	if debug:
		cv2.imshow('Circles',single channel)

	return get_circles(cont)

def draw_circles():
	for i in circles[0,:]:
		#draw the outer circle
		cv2.circle(file,(i[0],i[1],i[2],(0,255,0),2)
		#draw the center of the circle
		cv2.circle(file,(i[0],i[1]),2,(0,0,255),3)

#Image processing
image = get_from_file(filename)
cv2.imshow('raw',image)

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

lower_blue = np.array([100,50,50])
upper_blue = np.array([130,255,255])

lower_green = np.array([35,50,50])
upper_green = np.array([90,255,255])

lower_yellow = np.array([20,50,50])
upper_yellow = np.array([30,255,255])

lower_red = np.array([0,50,50])
upper_red = np.array([20,255,255])


blue_circles = do_full(image,hsv,upper_blue,lower_blue)
green_circles = do_full(image,hsv,upper_green,lower_green)
yellow_circles = do_full(image,hsv,upper_yellow,lower_yellow,True)
red_circles = do_full(image,hsv,upper_red,lower_red)

draw_circles(image,blue_circles,(255,0,0))
draw_circles(image,green_circles,(0,255,0))
draw_circles(image,yellow_circles,(0,255,255))
draw_circles(image,red_circles,(0,0,255))

#show resulting image
cv2.imshow('Result: Circles detected',image)
k = cv2.waitKey(0)
if k == 27:	#wait for ESC key to exit
	cv2.destroyAllWindows()
elif k == ord('s'):	#wait for the S-key being pressed to SAVE and EXIT
	cv2.imwrite(str(datetime.datetime.now()) + 'ProcessedImage.jpg',file)
	cv2.destroyAllWindows()



