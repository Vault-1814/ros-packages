import cv2
from enum import Enum
from configuration_menager import FilterInitValues


class FilterTitle(Enum):
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
        fields = FilterTitle.__members__.values()
        for f in fields:
            if title == f.value():
                return True
        return False


class Filter:
    _title = None
    _borderType = None

    def __init__(self, params):
        self._title = params.title
        self._borderType = params.borderType

    def apply(self, imageSrc):
        pass


class BilateralFilter(Filter):
    _d = None     # if negative then computed from sigmaSpace
    _sigmaColor = None
    _sigmaSpace = None

    def __init__(self, params):
        Filter.__init__(self, params)
        self._d = params.d
        self._sigmaColor = params.sigmaColor
        self._sigmaSpace = params.sigmaSpace

    def apply(self, imageSrc):
        return cv2.bilateralFilter(imageSrc, self._d, self._sigmaColor,
                                   self._sigmaSpace, None, self._borderType)

class Blur(Filter):
    _ksize = None
    _anchor = None

    def __init__(self, params):
        Filter.__init__(self, params)
        self._ksize = params.ksize
        self._anchor = params.anchor
        self._borderType = params.borderType

    def apply(self, imageSrc):
        return cv2.blur(imageSrc, self._ksize, None,
                        self._anchor, self._borderType)


class BoxFilter(Filter):
    _ddepth = None
    _ksize = None
    _anchor = None
    _normalize = None

    def __init__(self, params):
        Filter.__init__(self, params)
        self._ddepth = params.ddepth
        self._ksize = params.ksize
        self._anchor = params.anchor
        self._normalize = params.normalize

    def apply(self, imageSrc):
        return cv2.boxFilter(imageSrc, self._ddepth, self._ksize, None,
                             self._anchor, self._normalize, self._borderType)


class GaussianBlur(Filter):
    _ksize = None
    _sigmaX = None
    _sigmaY = None

    def __init__(self, params):
        Filter.__init__(self, params)
        self._ksize = params.ksize
        self._sigmaX = params.sigmaX
        self._sigmaY = params.sigmaY

    def apply(self, imageSrc):
        return cv2.GaussianBlur(imageSrc, self._ksize, self._sigmaX, None,
                                self._sigmaY, self._borderType)


class MedianBlur(Filter):
    _ksize = None

    def __init__(self, params):
        Filter.__init__(self, params)
        self._ksize = params.ksize

    def apply(self, imageSrc):
        return cv2.medianBlur(imageSrc, self._ksize)


"""
    Filter Parameters
"""
class FilterParameters:
    title = ''
    borderType = None


class BilateralFilterParameters(FilterParameters):
    d = None
    sigmaColor = None
    sigmaSpace = None


class BlurParameters(FilterParameters):
    ksize = None
    anchore = None


class BoxFilerParameters(FilterParameters:
    ddepth = None
    ksize = None
    anchor = None
    normalize = None


class GausssianBlurParameters(FilterParameters):
    _ksize = None
    _sigmaX = None
    _sigmaY = None


class MedianBlurParameters(FilterParameters):
    ksize = None


class FilterParametersFactory:
    instance = None

    def __new__(cls):
        if FilterFactory.instance is None:
            FilterFactory.instance = object.__new__(cls)
        return FilterFactory.instance

    def getParameters(self, filterTitle):
        pass

class NOOOOOOOOOOOFilterParameters(Parameters):
    def getParameters(self, filterTitle, mode):
        params = Parameters()
        if FilterTitle.isDefined(filterTitle):
            if mode: # configur by trackbars
                flrFilds = FilterFields.getFields(filterTitle)
                tbrWin = TrackbarWindow(filterTitle)
                tbrs = []
                for field in flrFilds:
                    initValues = TrackbarInitValues.getValues(field)
                    tbr = Trackbar(field, initValues)
                    tbrWin.addTrackbar(tbr)
                    params = tbrWin.getTrackbarVlues()
            else: # configure by config file
                flrInit = FilterInitValues()
                values = flrInit.getValues(filterTitle)
        return params


class FilterFactory(object):
    instance = None

    def __new__(cls):
        if FilterFactory.instance is None:
            FilterFactory.instance = object.__new__(cls)
        return FilterFactory.instance

    def __init__(self):
        self._filterParameters = FilterParameters()

    def getFilter(self, filterTitle, mode=1):
        if FilterTitle.isDefined(filterTitle):
            params = self._filterParameters.getParameters(filterTitle, mode)
            return {
                FilterTitle.BILATERAL_FILTER: FilterBuilder(filterTitle)????#BilateralFilter(params),
                FilterTitle.BLUR: Blur(params),
                FilterTitle.BOX_FILTER: BoxFilter(params),
                FilterTitle.GAUSSIAN_BLUR: GaussianBlur(params),
                FilterTitle.MEDIAN_BLUR: MedianBlur(params)
            }[filterTitle]

        return -1


class FilterBuilder:
    FilterParameters()
    Filter()


