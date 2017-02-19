import cv2
from enum import Enum

from trackbar_api import Field
from configuration_menager import FilterConfigReader


class FilterName(Enum):
    BILATERAL_FILTER = 'bilateralFilter'
    BLUR = 'blur'
    BOX_FILTER = 'boxFilter'
    GAUSSIAN_BLUR = 'gaussianBlur'
    MEDIAN_BLUR = 'medianBlur'

    def __init__(self, title):
        self.title = title

    def value(self):
        return self.title

    @staticmethod
    def isDefined(title):
        fields = FilterName.__members__.values()
        for f in fields:
            if title == f.value():
                return True
        return False


"""
    Filters Config
"""
class FilterConfig:
    title = ''
    borderType = None

    def __init__(self, title, config):
        self.title = title
        self.borderType = config[Field.BORDER_TYPE.value()]


class BilateralFilterConfig(FilterConfig):
    d = None
    sigmaColor = None
    sigmaSpace = None

    def __init__(self, config):
        FilterConfig.__init__(self, FilterName.BILATERAL_FILTER.value(), config)
        self.d = config[Field.D.value()]
        self.sigmaColor = config[Field.SIGMA_COLOR.value()]
        self.sigmaSpace = config[Field.SIGMA_SPACE.value()]


class BlurConfig(FilterConfig):
    ksize = None
    anchor = None

    def __init__(self, config):
        FilterConfig.__init__(self, FilterName.BLUR.value(), config)
        self.ksize = config[Field.KERNEL_SIZE.value()]
        self.anchor = config[Field.ANCHOR.value()]


class BoxFilterConfig(FilterConfig):
    ddepth = None
    ksize = None
    anchor = None
    normalize = None

    def __init__(self, config):
        FilterConfig.__init__(self, FilterName.BOX_FILTER.value(), config)
        self.ddepth = config[Field.DDEPTH.value()]
        self.ksize = config[Field.KERNEL_SIZE.value()]
        self.anchor = config[Field.ANCHOR.value()]
        self.normalize = config[Field.NORMALIZE.value()]


class GaussianBlurConfig(FilterConfig):
    ksize = None
    sigmaX = None
    sigmaY = None

    def __init__(self, config):
        FilterConfig.__init__(self, FilterName.GAUSSIAN_BLUR.value(), config)
        self.ksize = config[Field.KERNEL_SIZE.value()]
        self.sigmaX = config[Field.SIGMA_X.value()]
        self.sigmaY = config[Field.SIGMA_Y.value()]


class MedianBlurConfig(FilterConfig):
    ksize = None

    def __init__(self, config):
        FilterConfig.__init__(self, FilterName.MEDIAN_BLUR.value(), config)
        self.ksize = config[Field.KERNEL_SIZE.value()]


class FilterConfigFactory(object):

    def __init__(self):
        pass

    def getFilterConfig(self, filterName, mode):
        config = FilterConfigReader().read(filterName, mode)
        if FilterName.BILATERAL_FILTER.value() == filterName:
            return BilateralFilterConfig(config)
        elif FilterName.BLUR.value() == filterName:
            return BlurConfig(config)
        elif FilterName.BOX_FILTER.value() == filterName:
            return BoxFilterConfig(config)
        elif FilterName.GAUSSIAN_BLUR.value() == filterName:
            return GaussianBlurConfig(config)
        elif FilterName.MEDIAN_BLUR.value() == filterName:
            return MedianBlurConfig(config)


"""
    Filters
"""


class Filter:
    _title = None
    _borderType = None

    def __init__(self, config):
        self._title = config.title
        self._borderType = config.borderType

    def apply(self, imageSrc):
        pass


class BilateralFilter(Filter):
    _d = None     # if negative then computed from sigmaSpace
    _sigmaColor = None
    _sigmaSpace = None

    def __init__(self, config):
        Filter.__init__(self, config)
        self._d = config.d
        self._sigmaColor = config.sigmaColor
        self._sigmaSpace = config.sigmaSpace

    def apply(self, imageSrc):
        return cv2.bilateralFilter(imageSrc, self._d, self._sigmaColor,
                                   self._sigmaSpace, None, self._borderType)


class Blur(Filter):
    _ksize = None
    _anchor = None

    def __init__(self, config):
        Filter.__init__(self, config)
        self._ksize = config.ksize
        self._anchor = config.anchor
        self._borderType = config.borderType

    def apply(self, imageSrc):
        return cv2.blur(imageSrc, self._ksize, None, self._anchor, self._borderType)


class BoxFilter(Filter):
    _ddepth = None
    _ksize = None
    _anchor = None
    _normalize = None

    def __init__(self, config):
        Filter.__init__(self, config)
        self._ddepth = config.ddepth
        self._ksize = config.ksize
        self._anchor = config.anchor
        self._normalize = config.normalize

    def apply(self, imageSrc):
        return cv2.boxFilter(imageSrc, self._ddepth, self._ksize, None,
                             self._anchor, self._normalize, self._borderType)


class GaussianBlur(Filter):
    _ksize = None
    _sigmaX = None
    _sigmaY = None

    def __init__(self, config):
        Filter.__init__(self, config)
        self._ksize = config.ksize
        self._sigmaX = config.sigmaX
        self._sigmaY = config.sigmaY

    def apply(self, imageSrc):
        return cv2.GaussianBlur(imageSrc, self._ksize, self._sigmaX, None,
                                self._sigmaY, self._borderType)


class MedianBlur(Filter):
    _ksize = None

    def __init__(self, config):
        Filter.__init__(self, config)
        self._ksize = config.ksize

    def apply(self, imageSrc):
        return cv2.medianBlur(imageSrc, self._ksize)


class FilterFactory(object):
    instance = None

    def __new__(cls):
        if FilterFactory.instance is None:
            FilterFactory.instance = object.__new__(cls)
        return FilterFactory.instance

    def __init__(self):
        self.flrCfgFactory = FilterConfigFactory()

    def getFilter(self, filterName, mode=0):
        if FilterName.isDefined(filterName):
            print('getFilter: ' + filterName)
            flrConfig = self.flrCfgFactory.getFilterConfig(filterName, mode)
            if FilterName.BILATERAL_FILTER.value() == filterName:
                return BilateralFilter(flrConfig)
            elif FilterName.BLUR.value() == filterName:
                return Blur(flrConfig)
            elif FilterName.BOX_FILTER.value() == filterName:
                return BoxFilter(flrConfig)
            elif FilterName.GAUSSIAN_BLUR.value() == filterName:
                return GaussianBlur(flrConfig)
            elif FilterName.MEDIAN_BLUR.value() == filterName:
                return MedianBlur(flrConfig)
        return -1

if __name__ == '__main__':
    image = cv2.imread('/media/data/evo/py_projects/cv/raw_images/3.jpg')
    image = cv2.resize(image, (640, 480))

    flrFactory = FilterFactory()
    filterBlur = flrFactory.getFilter(FilterName.MEDIAN_BLUR.value())
    image = filterBlur.apply(image)

    while True:
        cv2.imshow('0', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break