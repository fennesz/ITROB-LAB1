import json
from threading import Thread,Event

class RecurringConfigReader(Thread):
    def __init__(self, event, configName):
        Thread.__init__(self)
        self.stopped = event
        self.configName = configName

    def run(self):
        while not self.stopped.wait(5):
            print("Reading config...")
            ConfigAccessor.data = FileReader.read_data(self.configName)
            print "Data in ReccuringConfigReader: " + str(ConfigAccessor.data)

class FileReader:
    @staticmethod
    def read_data(configName):
        with open(configName, 'r') as f:
            return json.load(f)

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
            ConfigAccessor.data = FileReader.read_data(self.configName)
        except IOError:
            self.__populate_default_data()
            FileWriter.write_data(self.configName, ConfigAccessor.data)
        finally:
            ConfigAccessor.stopFlag = Event()
            ConfigAccessor.confReader = RecurringConfigReader(ConfigAccessor.stopFlag, self.configName)
            ConfigAccessor.confReader.start()

    def __populate_default_data(self):
        ConfigAccessor.data = {
            'exposure' : 1
        }

    def __exit__(self):
        ConfigAccessor.stopFlag.set()

if __name__ == '__main__':
    cfgAccess = ConfigAccessor("tenderbot")