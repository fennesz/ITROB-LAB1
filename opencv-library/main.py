from retrieveImage import RetrieveImage
from debugger import Debugger
from configaccess import ConfigAccessor

'''DEBUG VALUES'''
DEBUG = True
FPS = 0.5
MS_PER_FRAME = 1 / FPS * 1000

imgrtriever = RetrieveImage()
cfg = ConfigAccessor('tenderbot')
while True:
    img = imgrtriever.get_from_webcam("http://nano.pse.umass.edu:81/axis-cgi/jpg/image.cgi?resolution=640x480", ConfigAccessor.data['exposure'])
    if (DEBUG):
        Debugger.show_image(img, MS_PER_FRAME)


