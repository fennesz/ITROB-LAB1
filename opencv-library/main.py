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
        img = imgrtriever.get_from_webcam("http://nano.pse.umass.edu:81/axis-cgi/jpg/image.cgi?resolution=640x480", cfg.data['exposure'])
        if (DEBUG):
            Debugger.show_image(img, FPS)
    except KeyboardInterrupt:
        ConfigAccessor.stopFlag.set()
        sys.exit(0)