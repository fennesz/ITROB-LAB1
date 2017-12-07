import cv2

from configaccess import ConfigAccessor


class Debugger:

    images_showing = 0
    @staticmethod
    def show_image(img, fps=-1, name="Debug", show_once=False):
        Debugger.images_showing += Debugger.images_showing
        cv2.imshow(name, img)
        if (show_once):
            key = cv2.waitKey()
        else:
            if Debugger.images_showing > 1:
                fps = 500 / Debugger.images_showing
            key = cv2.waitKey(int(fps))
            if key is not -1:
                exit(0)
