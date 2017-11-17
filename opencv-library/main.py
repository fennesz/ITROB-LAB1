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
            img = imgrtriever.get_from_webcam("http://nano.pse.umass.edu:81/axis-cgi/jpg/image.cgi?resolution=640x480", cfg.data['exposure'])
        else:
            img = imgrtriever.get_from_file(sys.argv[1])
        imgPreparer = PrepareImage(image=img, cfgAccessor=cfg)
        thresholdedImage = imgPreparer.threshold_image()
        objectRgn = ObjectRecognizer(thresholdedImage)
        circleImg = objectRgn.find_and_draw_circles()

        if (DEBUG):
            Debugger.show_image(circleImg, FPS)
    except Exception as ex:
        ConfigAccessor.stopFlag.set()
        raise
        sys.exit(0)