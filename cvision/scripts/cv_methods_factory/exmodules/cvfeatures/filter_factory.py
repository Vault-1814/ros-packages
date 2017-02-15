import rospy
import cv2
from .. import utils
from .. import cvgui

BLUR_FILTER = 'blur'
BILATERAL_FILTER = 'bilateral_filter'
GAUSSIAN_BLUR = 'gaussian_blur'
MEDIAN_BLUR = 'median_blur'
ERODE_FILTER = 'erode'


def getFilterName(num):
    return {
        0: BLUR_FILTER,
        1: BILATERAL_FILTER,
        2: GAUSSIAN_BLUR,
        3: MEDIAN_BLUR,
        4: ERODE_FILTER
    }[num]


class Filter:
    KERNEL_SIZE = 'ksize'
    DIAMETER_PX = 'd. px'
    SIGMA_COLOR = 'sigma color'
    SIGMA_SPACE = 'sigma space'
    SIGMA_X = 'sigma x'
    SIGMA_Y = 'sigma y'
    SHAPE_TYPE = 'shape type'
    ITERATIONS = 'iters'

    BORDER_TYPE = 'borderType'
    BORDER_TYPE_QTY = 8
    BORDER_TYPE_DEFAULT = 1

    def __init__(self, tbsWindow):
        self.title = ''
        self.tbsWindow = tbsWindow
        self.fborderType = cvgui.Trackbar(Filter.BORDER_TYPE,
                                          (0, Filter.BORDER_TYPE_QTY,
                                         Filter.BORDER_TYPE_DEFAULT))

    def filtering(self, image):
        pass


class Blur(Filter):

    def __init__(self, tbsWindow, k=(0, 100, 1)):
        Filter.__init__(self, tbsWindow)
        self.title = BLUR_FILTER
        self.fksize = cvgui.Trackbar(Filter.KERNEL_SIZE, k)
        self.fanchor = None
        self.tbsWindow.add_filters(self.fksize, self.fborderType)

    def filtering(self, image):
        ksize, borderType = self.tbsWindow.get_trackbar_values()
        ksize = (ksize, ksize)
        bt = utils.getBorderType(borderType)
        image = cv2.blur(image, ksize, borderType=bt)
        return image


class Bilateral(Filter):

    def __init__(self, tbsWindow, d=(0, 10, -1), sc=(0, 300, 0), ss=(0, 300, 0)):
        Filter.__init__(self, tbsWindow)
        self.title = BILATERAL_FILTER
        fd = cvgui.Trackbar(Filter.DIAMETER_PX, d)
        fsc = cvgui.Trackbar(Filter.SIGMA_COLOR, sc)
        fss = cvgui.Trackbar(Filter.SIGMA_SPACE, ss)
        self.tbsWindow.add_filters(fd, fsc, fss, self.fborderType)

    def filtering(self, image):
        """d=-1 for compute itself; d=5 for online; d=9 for offline"""
        d, sc, ss, borderType = self.tbsWindow.get_trackbar_values()
        bt = utils.getBorderType(borderType)
        # TODO check border type !!!
        image = cv2.bilateralFilter(image, d, sc, ss, borderType=bt)
        return image


class GaussianBlur(Filter):

    def __init__(self, tbsWindow, k=(0, 100, 1), sx=(0, 300, 0), sy=(0, 300, 0)):
        Filter.__init__(self, tbsWindow)
        self.title = GAUSSIAN_BLUR
        fksize = cvgui.Trackbar(Filter.KERNEL_SIZE, k)
        fsx = cvgui.Trackbar(Filter.SIGMA_X, sx)
        fsy = cvgui.Trackbar(Filter.SIGMA_Y, sy)
        self.tbsWindow.add_filters(fksize, fsx, fsy, self.fborderType)

    def filtering(self, image):
        k, sx, sy, borderType = self.tbsWindow.get_trackbar_values()
        if k % 2 == 1:
            ksize = (k, k)
        else:
            ksize = (k - 1, k - 1)
        bt = utils.getBorderType(borderType)
        image = cv2.GaussianBlur(image, ksize, sigmaX=sx, sigmaY=sy,
                                 borderType=bt)
        return image


class MedianBlur(Filter):

    def __init__(self, tbsWindow, k=(0, 20, 2)):
        Filter.__init__(self, tbsWindow)
        self.title = MEDIAN_BLUR
        fksize = cvgui.Trackbar(Filter.KERNEL_SIZE, k)
        self.tbsWindow.add_filter(fksize)

    def filtering(self, image):
        ksize = self.tbsWindow.get_trackbar_value(Filter.KERNEL_SIZE)
        if ksize % 2 == 0:
            ksize -= 1  # because settings of opencv trackbars are poor
        image = cv2.medianBlur(image, ksize)
        return image


class Erode(Filter):

    def __init__(self, tbsWindow, sh=(0, 2, 0), k=(0, 100, 1), its=(0, 50, 0)):
        Filter.__init__(self, tbsWindow)
        self.title = ERODE_FILTER
        fshape = cvgui.Trackbar(Filter.SHAPE_TYPE, sh)
        fk = cvgui.Trackbar(Filter.KERNEL_SIZE, k)
        fits = cvgui.Trackbar(Filter.ITERATIONS, its)
        self.tbsWindow.add_filters(fshape, fk, fits, self.fborderType)

    def filtering(self, image):
        sh, k, its, borderType = self.tbsWindow.get_trackbar_values()
        ksize = (k, k)
        bt = utils.getBorderType(borderType)
        morph = utils.getShape(sh)
        kernel = cv2.getStructuringElement(morph, ksize)
        image = cv2.erode(image, kernel, iterations=its, borderType=bt)
        return image


class FilterFactory:
    # TODO SINGLETON !!!
    def __init__(self):
        pass

    def getFilter(self, title, tbsWindow):
        rospy.loginfo(tbsWindow)
        filter = {
            BLUR_FILTER: Blur,
            BILATERAL_FILTER: Bilateral,
            GAUSSIAN_BLUR: GaussianBlur,
            MEDIAN_BLUR: MedianBlur,
            ERODE_FILTER: Erode
        }[title]
        rospy.loginfo(title + ' filter is returning!')
        return filter(tbsWindow)
