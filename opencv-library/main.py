from retrieveImage import RetrieveImage
from debugger import Debugger
from configaccess import ConfigAccessor
import sys

cfg = ConfigAccessor('tenderbot')

'''DEBUG VALUES'''
DEBUG = True
FPS = ConfigAccessor.data['webcamFPS']
MS_PER_FRAME = 1000 / FPS

imgrtriever = RetrieveImage()
while True:
    try:
        img = imgrtriever.get_from_webcam("http://nano.pse.umass.edu:81/axis-cgi/jpg/image.cgi?resolution=640x480", ConfigAccessor.data['exposure'])
        if (DEBUG):
            Debugger.show_image(img, MS_PER_FRAME)
    except KeyboardInterrupt:
        ConfigAccessor.stopFlag.set()
        sys.exit(0)