import cv2
from enum import Enum
from configuration_menager import FilterInitValues, TrackbarInitValues


class Field(Enum):
    KERNEL_SIZE = 'ksize'
    THRESHOLD_1 = 'threshold1'
    THRESHOLD_2 = 'threshold2'

    def __init__(self, title):
        self.title = title

    def value(self):
        return self.title

    @staticmethod
    def isDefined(title):
        fields = Field.__members__.values()
        for f in fields:
            if title == f.value():
                return True
        return False


class Trackbar:
    def __init__(self, title, (min, max, value)):
        self._title = title
        self._min = min
        self._max = max
        self._value = value

    def callback(self, value):
        self._value = value

    def getTitle(self):
        return self._title

    def getValue(self):
        return self._value

    def getMin(self):
        return self._min

    def getMax(self):
        return self._max


class TrackbarFactory(object):
    instance = None

    def __new__(cls):
        if TrackbarFactory.instance is None:
            TrackbarFactory.instance = object.__new__(cls)
        return TrackbarFactory.instance

    def getTrackbar(self, fieldName):
        if Field.isDefined(fieldName):
            initValues = TrackbarInitValues().getValues(fieldName)
            return Trackbar(fieldName, initValues)
        return -2


class TrackbarWindow:
    def __init__(self, winName):
        self.winName = winName
        cv2.namedWindow(self.winName)

        self.trackbars = {}

    def addTrackbar(self, trackbar):
        cv2.createTrackbar(trackbar.getTitle(), self.winName,
                           trackbar.getValue(), trackbar.getMax(), trackbar.callback)
        self.trackbars[trackbar.getTitle()] = trackbar

    def getTrackbarValue(self, trackbarTitle):
        tb = self.trackbars[trackbarTitle]
        return tb.getValue()

    def getTrackbarVlues(self):
        pass


if __name__ == '__main__':
    tbf = TrackbarFactory()
    tb = tbf.getTrackbar(Field.KERNEL_SIZE.value())
    tbw = TrackbarWindow('blur')
    tbw.addTrackbar(tb)

    while True:
        k = tbw.getTrackbarValue(Field.KERNEL_SIZE.value())
        print k
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break