#!/usr/bin/env python
import cv2
import numpy as np
import sys
import urllib
import errno
import os
from twisted.protocols.ftp import FileNotFoundError
import datetime

class RetrieveImage:
    BUFFER_SIZE = 64500

    def get_from_file(self, filename):
		print "Loading from file..."
		file = cv2.imread(filename, 1)
		if file is not None:
			print "Succesfully loaded file"
			return file
		else:
			print "Couldn't find file: " + filename
			raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), filename)

    def get_from_webcam(self, url, takes = 1):
        print "try fetch from webcam..."
        img = ''
        while takes > 0:
            takes -= 1
            stream=urllib.urlopen(url)
            bytes=''
            bytes+=stream.read(self.BUFFER_SIZE)
            a = bytes.find('\xff\xd8')
            b = bytes.find('\xff\xd9')
            if a != -1 and b != -1:
                jpg = bytes[a:b+2]
                bytes= bytes[b+2:]
                tmp = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.CV_LOAD_IMAGE_GRAYSCALE)
            else:
                raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), "Try increasing buffersize")
            if img is not '':
                img = cv2.add(img, tmp)
            else:
                img = tmp
        return img


def __fail_parameters():
    print "--------------------------------------------------"
    print sys.argv[0] + " takes between 2 and 4 parameters. Examples:"
    print sys.argv[0] + " --file image.jpg"
    print sys.argv[0] + " --webcam <takes/exposure> <url> "
    print "--------------------------------------------------"
    exit(1)

if __name__ == '__main__':
    retrImg = RetrieveImage()
    if len(sys.argv) < 2 or len(sys.argv) > 4:
        __fail_parameters()
    option = sys.argv[1]
    try:
        takes = sys.argv[2]
    except IndexError:
        takes = 1
    try:
        urlOrFilename = sys.argv[3]
    except IndexError:
        urlOrFilename = None
    takes = int(takes)
    if (option == "--file"):
        print "Loading " + urlOrFilename
        image = retrImg.get_from_file(urlOrFilename)
        cv2.imshow(urlOrFilename, image)
        while True:
            key = cv2.waitKey(0)
            if key is not None:
                exit(0)
    elif (option == "--webcam"):
        if (urlOrFilename == None):
            urlOrFilename = 'http://192.168.0.20/image/jpeg.cgi' #http://nano.pse.umass.edu:81/axis-cgi/jpg/image.cgi?resolution=640x480
        image = retrImg.get_from_webcam(urlOrFilename, takes)
        cv2.imwrite(str(datetime.datetime.now()) + '.jpg', image)
    else:
        __fail_parameters()
