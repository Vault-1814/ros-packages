from enum import Enum
import yaml
import os

PATH = '/home/kirix/catkin_ws/src/cvision/scripts/'

class ConfigFiles(Enum):
    TRACKBARS_INIT_FILE_NAME = PATH + 'resources/trackbars.yaml'
    FILTERS_INIT_FILE_NAME = PATH + 'resources/filters.yaml'

    def __init__(self, title):
        self.title = title

    def value(self):
        return self.title

class File:
    def __init__(self, file):
        self.file = file


class ConfigFileReader(File):
    def __init__(self, file):
        File.__init__(self, file)

    def read(self):
        raw = self.file.read(-1)
        data = yaml.load(raw)
        return data


class ConfigFileWriter(File):
    def __init__(self, file):
        File.__init__(self, file)

    def write(self):
        pass


class Values:
    def getValues(self, title):
        pass


class FilterInitValues(Values):
    """ read from config file parameters for certain filter"""
    def getValues(self, filterName):
        file = open(ConfigFiles.FILTERS_INIT_FILE_NAME, 'r')
        cfr = ConfigFileReader(file)
        flrs_param = cfr.read()
        flr_param_pairs = flrs_param[filterName]
        return flr_param_pairs.values()


class TrackbarInitValues(Values):
    """ read from config file init values for certain field-trackbar"""
    def getValues(self, fieldName):
        file = open(ConfigFiles.TRACKBARS_INIT_FILE_NAME.value(), 'r')
        cfr = ConfigFileReader(file)
        fields = cfr.read()
        return fields[fieldName]
