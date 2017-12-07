import cv2

from configaccess import ConfigAccessor


class Debugger:
    @staticmethod
    def show_image(img, fps=-1):
        cv2.imshow("Debug", img)
        key = cv2.waitKey(int(fps))
        if key is not -1:
            ConfigAccessor.stopFlag.set()
            exit(0)
