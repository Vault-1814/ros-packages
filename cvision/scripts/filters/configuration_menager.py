import yaml
from enum import Enum

from scripts.filters.filters import *

# TODO doing better it
PATH = '/home/kirix/catkin_ws/src/cvision/scripts/'


class ConfigFileName(Enum):
    FILTER_INIT_CONFIG_FILE_NAME = PATH + 'resources/filters.yaml'

    def __init__(self, title):
        self.title = title

    def value(self):
        return self.title


class File:
    def __init__(self, file):
        self.file = file


class YAMLFileReader(File):
    def __init__(self, file):
        File.__init__(self, file)

    def read(self):
        raw = self.file.read(-1)
        data = yaml.load(raw)
        return data


class YAMLFileWriter(File):
    def __init__(self, file):
        File.__init__(self, file)

    def write(self):
        pass


class ConfigReader:
    def __init__(self):
        pass

    def read(self, name, mode):
        file = open(ConfigFileName.FILTER_INIT_CONFIG_FILE_NAME.value(), 'r')
        cfr = YAMLFileReader(file)
        params = cfr.read()
        return params


class FilterConfigReader(ConfigReader):
    _FILTER_FIELD = 'filter'
    _TRACKBAR_FIELD = 'trackbar'

    """ read from config file parameters for certain filter"""
    def read(self, filterName, mode):
        params = ConfigReader.read(self, filterName, mode)
        if mode:
            return params[filterName][self._TRACKBAR_FIELD]
        else:
            return params[filterName][self._FILTER_FIELD]

if __name__ == '__main__':
    fic = FilterConfigReader()
    p = fic.read(FilterName.BLUR.value(), 0)
    print(p)
