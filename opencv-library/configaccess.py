import json
import os
from threading import Thread,Event

class RecurringConfigReader(Thread):
    configUpdateRate = 5

    def __init__(self, event, configName):
        Thread.__init__(self)
        self.fileReader = FileReader(configName)
        self.stopped = event

    def run(self):
        while not self.stopped.wait(self.configUpdateRate):
            ConfigAccessor.data = self.fileReader.read_data()
            self.configUpdateRate = ConfigAccessor.data['configUpdateRate']
            print "Data in ReccuringConfigReader: " + str(ConfigAccessor.data)


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

    def __init__(self, configName):
        ConfigAccessor.configName = configName + ".json"
        try:
            ConfigAccessor.data = FileReader(ConfigAccessor.configName).read_data()
        except OSError:
            self.__populate_default_data()
            FileWriter.write_data(self.configName, ConfigAccessor.data)
        finally:
            ConfigAccessor.stopFlag = Event()
            ConfigAccessor.confReader = RecurringConfigReader(ConfigAccessor.stopFlag, self.configName)
            ConfigAccessor.confReader.start()

    def setValue(self, key, value):
        ConfigAccessor.data[key] = value
        FileWriter.write_data(self.configName, ConfigAccessor.data)

    def __populate_default_data(self):
        ConfigAccessor.data = {
            'exposure' : 1,
            'configUpdateRate': 5,
	        'webcamFPS': 5000.0,
            'isDebug': False,
            'thresholdValue': 50,
        }

    def __exit__(self):
        ConfigAccessor.stopFlag.set()

if __name__ == '__main__':
    cfgAccess = ConfigAccessor("tenderbot")