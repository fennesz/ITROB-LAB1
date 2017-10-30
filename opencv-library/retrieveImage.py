#!/usr/bin/env python
import cv2
import numpy as np
import sys
import urllib
import errno
import os
from twisted.protocols.ftp import FileNotFoundError
import datetime

class retrieve_image:
    BUFFER_SIZE = 64500

    def get_from_file(self, filename):
		print "Loading from file..."
		file = cv2.imread(filename)
		if file is not None:
			print "Succesfully loaded file"
			return file
		else:
			print "Couldn't find file: " + filename
			raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), filename)

    def get_from_webcam(self, url):
        print "try fetch from webcam..."
        stream=urllib.urlopen(url)
        bytes=''
        bytes+=stream.read(self.BUFFER_SIZE)
        a = bytes.find('\xff\xd8')
        b = bytes.find('\xff\xd9')
        if a != -1 and b != -1:
            jpg = bytes[a:b+2]
            bytes= bytes[b+2:]
            i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.CV_LOAD_IMAGE_COLOR)
            return i
        else:
            raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), "Try increasing buffersize")


def __fail_parameters():
    print "--------------------------------------------------"
    print sys.argv[0] + " takes 2 or 3 parameters. Examples:"
    print sys.argv[0] + " --file image.jpg"
    print sys.argv[0] + " --webcam <url>"
    print "--------------------------------------------------"
    exit(1)

if __name__ == '__main__':
	#print len(sys.argv)
	retrImg = retrieve_image()
	if len(sys.argv) < 2 or len(sys.argv) > 3:
		__fail_parameters()
	option = sys.argv[1]
	try:
		urlOrFilename = sys.argv[2]
	except IndexError:
		urlOrFilename = None
	if (option == "--file"):
		print "Loading " + urlOrFilename
		image = retrImg.get_from_file(urlOrFilename)
		cv2.imshow(urlOrFilename, image)
		while True:
		    key = cv2.waitKey(0)
		    if key == 27: #27 is ESC
		        exit(0)
	elif (sys.argv[1] == "--webcam"):
		if (urlOrFilename == None):
			urlOrFilename = 'http://192.168.0.20/image/jpeg.cgi'
		image = retrImg.get_from_webcam(urlOrFilename)
		cv2.imwrite(str(datetime.datetime.now()) + '.jpg', image)
	else:
		__fail_parameters()
