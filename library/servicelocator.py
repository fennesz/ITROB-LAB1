from library.configaccess import ConfigAccessor


class ServiceLocator():

    @staticmethod
    def get_config(filename='tenderbot'):
        return ConfigAccessor(filename)