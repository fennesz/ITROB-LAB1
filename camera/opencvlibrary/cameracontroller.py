from debugger import Debugger
from objectrecognizer import ObjectRecognizer
from prepareimage import PrepareImage
from retrieveImage import RetrieveImage


class CameraController:
    def __init__(self, cfgAccess):
        self.cfgAccess = cfgAccess
        self.cfgAccess.start_reading()

    def get_raw_webcam_image(self, url='http://192.168.0.20/image/jpeg.cgi', fromFile=False):
        imgRetr = RetrieveImage()
        if fromFile:
            return imgRetr.get_from_file('src/ITROB-LAB1/camera/opencvlibrary/testimages/full_light_many_shapes.jpg')
        return imgRetr.get_from_webcam(url, takes=self.cfgAccess.data['exposure'])

    def get_shapes(self, image, color=None, Debug=True):
        if color is None:
            color = self.cfgAccess.data['circleColor']
        imgPreparer = PrepareImage(image=image, cfgAccessor=self.cfgAccess)
        single_channelImg = imgPreparer.extract_single_color_range(color=color)
        greyImg = imgPreparer.greyscale(single_channelImg)
        treshholded_img = imgPreparer.threshold_image(image=greyImg)
        objectRgn = ObjectRecognizer(treshholded_img, cfgAccessor=self.cfgAccess)
        circles = objectRgn.find_circles(treshholded_img)
        if Debug:
            lol = objectRgn.draw_circles(circles=circles, image=image)
            Debugger.show_image(lol, fps=self.cfgAccess.data['webcamFPS'])
            Debugger.show_image(treshholded_img, fps=self.cfgAccess.data['webcamFPS'], name="DebugTreshold")
        return circles

    def stop(self):
        self.cfgAccess.stop_reading()