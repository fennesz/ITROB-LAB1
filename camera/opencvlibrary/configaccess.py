import json
import numpy as np
import os
import sys
from threading import Timer

class FileReader:
    def __init__(self, configName):
        self.configName = configName
        self.modifiedDate = os.path.getmtime(configName)
        with open(self.configName, 'r') as f:
            self.data = json.load(f)

    def read_data(self):
        updatedDate = os.path.getmtime(self.configName)
        if updatedDate == self.modifiedDate:
            return self.data
        with open(self.configName, 'r') as f:
            print "The configfile changed, reloading values"
            self.data = json.load(f)
            self.modifiedDate = updatedDate
            return self.data


class FileWriter:
    @staticmethod
    def write_data(configName, data):
        with open(configName, 'w') as outfile:
            json.dump(data, outfile)

class ConfigAccessor:
    data = {}
    timer = None
    reader = None

    def __init__(self, configName, location=''):
        if location == 'catkin':
            configName = '../../../../' + configName
        self.configName = configName + ".json"
        print "Config path: " + self.configName
        try:       
            self.reader = FileReader(self.configName)
        except OSError:
            self.__populate_default_data()
            FileWriter.write_data(self.configName, self.data)
        
    def start_reading(self):
        try:       
            if self.reader == None:
                self.reader = FileReader(self.configName)            
            self.data = self.reader.read_data()
            print(self.data)
        except OSError:
            self.__populate_default_data()
            FileWriter.write_data(self.configName, self.data)
        self.timer = Timer(self.data["configUpdateRate"], lambda: self.start_reading())
        self.timer.start()
    
    def stop_reading(self):
        print "stop_reading called"
        if self.timer != None:
            self.timer.cancel()
            self.timer = None

    def setValue(self, key, value):
        self.data[key] = value
        FileWriter.write_data(self.configName, self.data)

    def __populate_default_data(self):
        self.data = {
            'exposure' : 1,
            'configUpdateRate': 5,
	        'webcamFPS': 5000.0,
            'isDebug': True,
            'thresholdValue': 50,
            'dp': 1,
            'minDistance': 20,
            'minRadius': 0,
            'maxRadius': 120,
            'param1': 100,
            'param2': 10,
            'dilatePixelsX': 3,
            'dilatePixelsY': 3,
            'closePixelsX': 5,
            'closePixelsY': 5,
            'circleColor': 'all'
        }

    def __exit__(self):
        print "destructor configaccess called"
        self.stop_reading()

if __name__ == '__main__':
    cfgAccess = ConfigAccessor("tenderbot")
