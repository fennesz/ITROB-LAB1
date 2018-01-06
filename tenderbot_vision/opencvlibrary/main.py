import numpy as np

from objectrecognizer import ObjectRecognizer
from prepareimage import PrepareImage
from retrieveImage import RetrieveImage
from debugger import Debugger
from configaccess import ConfigAccessor
import sys

CONFIG_FILENAME = 'tenderbot'

cfg = ConfigAccessor(CONFIG_FILENAME)
cfg.start_reading()

'''DEBUG VALUES'''
DEBUG = True

imgrtriever = RetrieveImage()
while True:
    try:
        DEBUG = cfg.data['isDebug']
        FPS = float(cfg.data['webcamFPS'])
        if (len(sys.argv) < 2):
            #img = imgrtriever.get_from_file('testimages/full_light_many_shapes.jpg')
            img = imgrtriever.get_from_webcam(takes=cfg.data['exposure'])
        else:
            img = imgrtriever.get_from_file(sys.argv[1])
        imgPreparer = PrepareImage(image=img, cfgAccessor=cfg)
        #TODO: Extract single color range, to create pseudo-8bit config
        single_channelImg = imgPreparer.extract_single_color_range(color=cfg.data['circleColor'])
        greyImg = imgPreparer.greyscale(single_channelImg)
        treshholded_img = imgPreparer.threshold_image(image=greyImg)
        objectRgn = ObjectRecognizer(treshholded_img, cfgAccessor=cfg)
        circles = objectRgn.find_circles(treshholded_img);
        if (circles is not None):
            objectRgn.draw_circles(circles, img)
        circleImg = objectRgn.find_and_draw_circles()
        if (DEBUG):
            Debugger.show_image(single_channelImg, FPS, "Debug_SingleChannel")
            Debugger.show_image(circleImg, FPS, "Debug_Treshold")
            Debugger.show_image(img, FPS, "Debug_Color")
    except Exception as ex:
        cfg.stop_reading()
        raise
        sys.exit(0)
