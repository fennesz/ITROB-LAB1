from objectrecognizer import ObjectRecognizer
from prepareimage import PrepareImage
from retrieveImage import RetrieveImage
from debugger import Debugger
from configaccess import ConfigAccessor
import sys

CONFIG_FILENAME = 'tenderbot'

cfg = ConfigAccessor(CONFIG_FILENAME)

'''DEBUG VALUES'''
DEBUG = True

imgrtriever = RetrieveImage()
while True:
    try:
        DEBUG = cfg.data['isDebug']
        FPS = float(cfg.data['webcamFPS'])
        if (len(sys.argv) < 2):
            img = imgrtriever.get_from_webcam("http://192.168.0.20/image/jpeg.cgi", cfg.data['exposure'])
        else:
            img = imgrtriever.get_from_file(sys.argv[1])
        imgPreparer = PrepareImage(image=img, cfgAccessor=cfg)
        treshholded_img = imgPreparer.threshold_image(img)
        objectRgn = ObjectRecognizer(treshholded_img, cfgAccessor=cfg)
        circleImg = objectRgn.find_and_draw_circles()

        if (DEBUG):
            Debugger.show_image(circleImg, FPS)
    except Exception as ex:
        ConfigAccessor.stopFlag.set()
        raise
        sys.exit(0)