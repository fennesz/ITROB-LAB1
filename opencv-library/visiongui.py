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

        self.app.setGeometry("500x200") #Size of window
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

        self.app.addButton("Save", func=self.saveValues)

    def onExit(self):
        self.cfg.stopFlag.set()
        return True

    def onIntervalChange(self, title):
        value = self.app.getScale("webcamFPS") / 10000.0
        if(value < 0.33): # 0-33% = 1-100
            value = value * 100
        elif(value < 0.66):# 33-66% = 100-1000
            value = value * 1000
        elif(value < 1.001):# 66-100% = 1000-10000
            value = value * 10000

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
        self.cfg.stopFlag.set()

    def saveValues(self, arg):
        cfg.setValue('exposure', self.exposureValue.get())
        cfg.setValue('configUpdateRate', self.configUpdateRateValue.get())
        cfg.setValue('webcamFPS', float(self.webcamFPSValue.get()))
        cfg.setValue('isDebug', self.app.getCheckBox("isDebug"))

if __name__ == '__main__':
    cfg = ConfigAccessor('tenderbot')
    vsgui = VisionGui(cfg)
    vsgui.execute()
