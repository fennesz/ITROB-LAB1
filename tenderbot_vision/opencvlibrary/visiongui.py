import math

import sys

from appJar import gui
from configaccess import ConfigAccessor

class VisionGui:
    def __init__(self, cfg):
        self.cfg = cfg
        self.modifiedScaleValue = 1

        self.populateStartValues()
        self.app = gui()
        self.app.setFont(10)

        self.app.setGeometry("900x700") #Size of window
        self.app.addLabel("title", "'tenderBot configurator", row=0, colspan=2)
        self.app.addButton("Save", func=self.saveValues)
        #self.app.addHorizontalSeparator(colspan=3, row=1)
        self.app.addLabel("exposure", "Exposure: ", 2, 0)
        self.exposureValue = self.app.addScale("exposure", 2, 1)
        self.app.setScaleRange("exposure", 1, 7, self.exposureValueDefault)
        self.app.showScaleIntervals("exposure", 1)


        self.app.addHorizontalSeparator(colspan=3, row=3)

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

        self.app.addLabel("lowBlue1", "Lower Blue param1:", 3, 2)
        self.paramlowBlue1Value = self.app.addScale("lowBlue1", 3, 3)
        self.app.setScaleRange("lowBlue1", 1, 100, self.paramlowBlue1ValueDefault)
        self.app.showScaleValue("lowBlue1", show=True)

        self.app.addLabel("upBlue1", "Upper Blue param1:", 4, 2)
        self.paramclosePixelsValue = self.app.addScale("upBlue1", 4, 3)
        self.app.setScaleRange("upBlue1", 1, 200, self.paramupBlue1ValueDefault)
        self.app.showScaleValue("upBlue1", show=True)

        self.app.addLabel("lowGreen1", "Lower Green param1:", 5, 2)
        self.paramlowGreen1Value = self.app.addScale("lowGreen1", 5, 3)
        self.app.setScaleRange("lowGreen1", 1, 50, self.paramlowGreen1ValueDefault)
        self.app.showScaleValue("lowGreen1", show=True)

        self.app.addLabel("upGreen1", "Upper Green param1:", 6, 2)
        self.paramupGreen1Value = self.app.addScale("upGreen1", 6, 3)
        self.app.setScaleRange("upGreen1", 1, 100, self.paramupGreen1ValueDefault)
        self.app.showScaleValue("upGreen1", show=True)

        self.app.addLabel("lowYel1", "Lower Yellow param1:", 7, 2)
        self.paramlowYel1Value = self.app.addScale("lowYel1", 7, 3)
        self.app.setScaleRange("lowYel1", 1, 50, self.paramlowYel1ValueDefault)
        self.app.showScaleValue("lowYel1", show=True)

        self.app.addLabel("upYel1", "Upper Yellow param1:", 8, 2)
        self.paramupYel1Value = self.app.addScale("upYel1", 8, 3)
        self.app.setScaleRange("upYel1", 1, 50, self.paramupYel1ValueDefault)
        self.app.showScaleValue("upYel1", show=True)

        self.app.addLabel("lowRed1", "Lower Red param1:", 9, 2)
        self.paramlowRed1Value = self.app.addScale("lowRed1", 9, 3)
        self.app.setScaleRange("lowRed1", 1, 50, self.paramlowRed1ValueDefault)
        self.app.showScaleValue("lowRed1", show=True)

        self.app.addLabel("upRed1", "Upper Red param1:", 10, 2)
        self.paramupRed1Value = self.app.addScale("upRed1", 10, 3)
        self.app.setScaleRange("upRed1", 1, 50, self.paramupRed1ValueDefault)
        self.app.showScaleValue("upRed1", show=True)

        self.app.addLabel("colors", "Color of circle:", 11, 2)
        self.paramColorsValue = self.app.addOptionBox("colors", ["all", "green", "blue",
                                          "yellow", "red"], 11, 3, self.getColorIndex(self.paramColorsValueDefault))
        self.app.setOptionBox("colors", index=self.getColorIndex(self.paramColorsValueDefault), callFunction=False)

        self.app.setStopFunction(self.onExit)

    def onExit(self):
        return True

    def getColorIndex(self, color):
        if color == 'all':
            return 0
        if color == 'green':
            return 1
        if color == 'blue':
            return 2
        if color == 'yellow':
            return 3
        if color == 'red':
            return 4

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
        self.paramlowBlue1ValueDefault = cfg.data['lowBlue1']
        self.paramupBlue1ValueDefault = cfg.data['upBlue1']
        self.paramlowGreen1ValueDefault = cfg.data['lowGreen1']
        self.paramupGreen1ValueDefault = cfg.data['upGreen1']
        self.paramlowYel1ValueDefault = cfg.data['lowYel1']
        self.paramupYel1ValueDefault = cfg.data['upYel1']
        self.paramlowRed1ValueDefault = cfg.data['lowRed1']
        self.paramupRed1ValueDefault = cfg.data['upRed1']
        self.paramColorsValueDefault = cfg.data['circleColor']

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
        cfg.setValue('lowBlue1', self.paramlowBlue1Value.get())
        cfg.setValue('upBlue1', self.paramupBlue1Value.get())
        cfg.setValue('lowGreen1', self.paramlowGreen1Value.get())
        cfg.setValue('upGreen1', self.paramupGreen1Value.get())
        cfg.setValue('lowYel1', self.paramlowYel1Value.get())
        cfg.setValue('upYel1', self.paramupYel1Value.get())
        cfg.setValue('lowRed1', self.paramlowRed1Value.get())
        cfg.setValue('upRed1', self.paramupRed1Value.get())
        cfg.setValue('circleColor', self.app.getOptionBox("colors"))

    def __exit__(self):
        pass	

if __name__ == '__main__':
    try:
        if len(sys.argv) > 1:
	    if sys.argv[1] == "ros":
	        cfg = ConfigAccessor('tenderbot', location='catkin')
	        del sys.argv[1]
        else:
            cfg = ConfigAccessor('tenderbot')
        cfg.start_reading()
        vsgui = VisionGui(cfg)
        vsgui.execute()
        print("After vsgui.execute()")
        cfg.stop_reading()
    except KeyboardInterrupt,SystemExit:
        cfg.stop_reading()
