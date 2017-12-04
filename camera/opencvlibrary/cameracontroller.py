from configaccess import ConfigAccessor
from retrieveImage import RetrieveImage


class CameraController:

    def __init__(self, cfgAccess):
        self.cfgAccess = cfgAccess

    def get_raw_webcam_image(self, url='http://192.168.0.20/image/jpeg.cgi', fromFile=False):
        imgRetr = RetrieveImage()
        if fromFile:
            return imgRetr.get_from_file('src/ITROB-LAB1/camera/opencvlibrary/testimages/full_light_many_shapes.jpg')
        return imgRetr.get_from_webcam(url, takes=self.cfgAccess.data['exposure'])

    def stop(self):
        self.cfgAccess.stopFlag.set()
