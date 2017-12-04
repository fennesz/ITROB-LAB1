import sys
import math

from appJar import gui
from configaccess import ConfigAccessor

class VisionGui:
    def __init__(self, cfg):
        self.cfg = cfg
        self.modifiedScaleValue = 1

        self.populateStartValues()
        self.app = gui()
        self.app.setFont(10)

        self.app.setGeometry("500x700") #Size of window
        self.app.addLabel("title", "'tenderBot configurator", row=0, colspan=2)
        self.app.addHorizontalSeparator(colspan=2, row=1)
        self.app.addLabel("exposure", "Exposure: ", 2, 0)
        self.exposureValue = self.app.addScale("exposure", 2, 1)
        self.app.setScaleRange("exposure", 1, 7, self.exposureValueDefault)
        self.app.showScaleIntervals("exposure", 1)

        self.app.addHorizontalSeparator(colspan=2, row=3)

        self.app.addLabel("configUpdateRate", "ConfigUpdateInterval (Reads per second): ", 3, 0)
        self.configUpdateRateValue = self.app.addScale("configUpdateRate", 3, 1)
        self.app.setScaleRange("configUpdateRate", 1, 600, self.configUpdateRateValueDefault)
        self.app.showScaleValue("configUpdateRate", show=True)

        self.app.addHorizontalSeparator(colspan=2, row=4)

        self.app.addLabel("webcamFPS", "Webcam Interval (MS): ", 4, 0)
        self.webcamFPSValue = self.app.addScale("webcamFPS", 4, 1)
        self.app.setScaleRange("webcamFPS", 1, 10000, self.webcamFPSValueDefault)
        self.app.showScaleValue("webcamFPS", show=True)
        #Below can be removed (Attempt to scale depending on scale value)
        #self.app.setScaleChangeFunction("webcamFPS", self.onIntervalChange)

        self.app.addLabel("isDebug", "Debug: ", 5, 0)
        self.debugValue = self.app.addCheckBox("isDebug", 5, 1)
        self.app.setCheckBox("isDebug", ticked=self.debugValueDefault)

        self.app.addLabel("thresholdValue", "Threshold Value: ", 6, 0)
        self.thresholdValue = self.app.addScale("thresholdValue", 6, 1)
        self.app.setScaleRange("thresholdValue", 5, 255, self.thresholdValueDefault)
        self.app.showScaleValue("thresholdValue", show=True)

        self.app.addLabel("dpValue", "Webcam DP*100:", 7, 0)
        self.dpValue = self.app.addScale("dpValue", 7, 1)
        self.app.setScaleRange("dpValue", 1, 500, self.dpValueDefault)
        self.app.showScaleValue("dpValue", show=True)

        self.app.addLabel("minRadius", "Circle minRadius:", 8, 0)
        self.minRadiusValue = self.app.addScale("minRadius", 8, 1)
        self.app.setScaleRange("minRadius", 1, 50, self.minRadiusValueDefault)
        self.app.showScaleValue("minRadius", show=True)

        self.app.addLabel("maxRadius", "Circle maxRadius:", 9, 0)
        self.maxRadiusValue = self.app.addScale("maxRadius", 9, 1)
        self.app.setScaleRange("maxRadius", 20, 150, self.maxRadiusValueDefault)
        self.app.showScaleValue("maxRadius", show=True)

        self.app.addLabel("param1", "Circle param1:", 10, 0)
        self.paramOneValue = self.app.addScale("param1", 10, 1)
        self.app.setScaleRange("param1", 1, 500, self.paramOneValueDefault)
        self.app.showScaleValue("param1", show=True)

        self.app.addLabel("param2", "Circle param2:", 11, 0)
        self.paramTwoValue = self.app.addScale("param2", 11, 1)
        self.app.setScaleRange("param2", 1, 500, self.paramTwoValueDefault)
        self.app.showScaleValue("param2", show=True)

        self.app.addLabel("minDistance", "Circle minDistance:", 12, 0)
        self.paramMinDistanceValue = self.app.addScale("minDistance", 12, 1)
        self.app.setScaleRange("minDistance", 1, 500, self.paramminDistanceValueDefault)
        self.app.showScaleValue("minDistance", show=True)

        self.app.addLabel("dilatePixels", "Dilate Pixels treshold:", 13, 0)
        self.paramDilatePixelsValue = self.app.addScale("dilatePixels", 13, 1)
        self.app.setScaleRange("dilatePixels", 1, 20, self.paramdilatePixelsValueDefault)
        self.app.showScaleValue("dilatePixels", show=True)

        self.app.addLabel("closePixels", "Close Pixels treshold:", 14, 0)
        self.paramclosePixelsValue = self.app.addScale("closePixels", 14, 1)
        self.app.setScaleRange("closePixels", 1, 20, self.paramclosePixelsValueDefault)
        self.app.showScaleValue("closePixels", show=True)

        self.app.addButton("Save", func=self.saveValues)

    def onExit(self):
        self.cfg.stopFlag.set()
        return True

    def onIntervalChange(self, title):
        value = t * 1.0
        if (value < 3333):
            value = ((100 / 3333) * value)
        elif (value < 6666):
            value = ((300 / 1111) * value) - 800
        elif (value <= 10000):
            value = ((4500 / 1667) * value) - (28330000 / 1667)
            self.modifiedScaleValue = self.nearestToCurrentOrderOfMagnitude(value)
            self.app.setLabel("webcamFPS", self.modifiedScaleValue)
        return


    def nearestToCurrentOrderOfMagnitude(self, t):
        power = math.floor(math.log10(t))
        nearest = math.floor(t / math.pow(10, power)) * math.pow(10, power)
        return nearest


    def execute(self):
        self.app.setStopFunction(self.onExit)
        self.app.go()

    def populateStartValues(self):
        self.exposureValueDefault = cfg.data['exposure']
        self.configUpdateRateValueDefault = cfg.data['configUpdateRate']
        self.webcamFPSValueDefault = cfg.data['webcamFPS']
        self.debugValueDefault = cfg.data['isDebug']
        self.thresholdValueDefault = cfg.data['thresholdValue']
        self.dpValueDefault = int(cfg.data['dp'] * 100)
        self.minRadiusValueDefault = cfg.data['minRadius']
        self.maxRadiusValueDefault = cfg.data['maxRadius']
        self.paramOneValueDefault = cfg.data['param1']
        self.paramTwoValueDefault = cfg.data['param2']
        self.paramminDistanceValueDefault = cfg.data['minDistance']
        self.paramdilatePixelsValueDefault = cfg.data['dilatePixelsX']
        self.paramclosePixelsValueDefault = cfg.data['closePixelsX']
        self.cfg.stopFlag.set()

    def saveValues(self, arg):
        cfg.setValue('exposure', self.exposureValue.get())
        cfg.setValue('configUpdateRate', self.configUpdateRateValue.get())
        cfg.setValue('webcamFPS', float(self.webcamFPSValue.get()))
        cfg.setValue('isDebug', self.app.getCheckBox("isDebug"))
        cfg.setValue('thresholdValue', self.thresholdValue.get())
        cfg.setValue('dp', float(self.dpValue.get()) / 100)
        cfg.setValue('minRadius', self.minRadiusValue.get())
        cfg.setValue('maxRadius', self.maxRadiusValue.get())
        cfg.setValue('param1', self.paramOneValue.get())
        cfg.setValue('param2', self.paramTwoValue.get())
        cfg.setValue('minDistance', self.paramMinDistanceValue.get())
        cfg.setValue('closePixelsX', self.paramclosePixelsValue.get())
        cfg.setValue('closePixelsY', self.paramclosePixelsValue.get())
        cfg.setValue('dilatePixelsX', self.paramDilatePixelsValue.get())
        cfg.setValue('dilatePixelsY', self.paramDilatePixelsValue.get())



if __name__ == '__main__':
    cfg = ConfigAccessor('tenderbot')
    vsgui = VisionGui(cfg)
    vsgui.execute()
