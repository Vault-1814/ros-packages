import rospy
import cv2
import imutils
import scripts.cv_methods_factory.exmodules.cvgui as tb
import scripts.cv_methods_factory.exmodules.utils
import threading
from collections import deque
import time
from sensor_msgs.msg import Image


class RefreshTrackbarsThread(threading.Thread):
    def __init__(self, queue, cfgFilteringNode):
        threading.Thread.__init__(self)
        self.queue = queue
        self.cfn = cfgFilteringNode

    def run(self):
        while True:
            while len(self.queue) == 0:
                time.sleep(0.1)
            self.cfn.typeBlur = self.cfn.tbTB.get_trackbar_value('TYPE_BLUR')
            if self.cfn.typeBlur != self.cfn.oldTypeBlur:
                if self.cfn.tbs:
                    cv2.destroyWindow(self.cfn.tbs.get_win_name())
                self.cfn.tbs = tb.Trackbar(self.cfn.getFilterName(self.cfn.typeBlur))
                self.cfn.tbs.add_filters(self.cfn.getTrackbarsForFilter(self.cfn.typeBlur))
                self.cfn.tbs.add_filter(self.cfn.fBorderType)
                self.cfn.oldTypeBlur = self.cfn.typeBlur
            k = cv2.waitKey(6) & 0xFF
            if k in [27, ord('q')]:
                rospy.signal_shutdown('Quit')
            elif k == ord('s'):
                # TODO save parameters for filter
                #self.cfgFilteringNode.parametersDump(lalalal)
                pass

class CfgFilteringNode:
    def __init__(self, cameraTopic, dstTopic):
        rospy.loginfo(cameraTopic)
        self.fTypeBlur = tb.Filter('TYPE_BLUR', 0, 8, 0)
        self.tbTB = tb.Trackbar('SELECT_TYPE_BLUR_FILTER', self.fTypeBlur)
        self.fBorderType = tb.Filter('BORDER_TYPE', 0, 8, 0)

        self.oldTypeBlur = -1
        self.typeBlur = 0
        self.tbs = ''

        self.subCamera = rospy.Subscriber(
            cameraTopic, Image, self.callback, 1)
        self.pubImage = rospy.Publisher(
            dstTopic, Image, queue_size=1)

        self.queueRefresh = deque([], 1)
        self.displayThread = RefreshTrackbarsThread(self.queueRefresh, self)
        self.displayThread.setDaemon(True)
        self.displayThread.start()

    def getBorderType(self, bt):
        """for opencv 3.2"""
        bts = (
            cv2.BORDER_CONSTANT,
            cv2.BORDER_REPLICATE,
            cv2.BORDER_REFLECT,
            cv2.BORDER_WRAP,
            cv2.BORDER_REFLECT_101,
            cv2.BORDER_TRANSPARENT,
            cv2.BORDER_REFLECT101,
            cv2.BORDER_DEFAULT,
            cv2.BORDER_ISOLATED)
        bts_name = (
            'BORDER_CONSTANT',
            'BORDER_REPLICATE',
            'BORDER_REFLECT',
            'BORDER_WRAP',
            'BORDER_REFLECT_101',
            'BORDER_TRANSPARENT',
            'BORDER_REFLECT101',
            'BORDER_DEFAULT',
            'BORDER_ISOLATED')
        #print(bts_name[bt], ' was selected!')
        return bts[bt]

    def getFilterName(self, typeBlur):
        if typeBlur == 0:
            return 'bilateral filter'
        elif typeBlur == 1:
            return 'blur'
        elif typeBlur == 2:
            return 'box filter'
        elif typeBlur == 3:
            return 'build pyramid'
        elif typeBlur == 4:
            return 'erode'
        elif typeBlur == 5:
            return 'filter2D'
        elif typeBlur == 6:
            return 'GaussianBlur'
        elif typeBlur == 7:
            return 'medianBlur'
        elif typeBlur == 8:
            return 'Laplacian'

    def getTrackbarsForFilter(self, typeBlur):
        if typeBlur == 0:
            fd = tb.Filter('DIAMETER_PX', 0, 10, -1)
            fsigC = tb.Filter('SIG_COLOR', 0, 255, 0)
            fsigS = tb.Filter('SIG_SPACE', 0, 255, 0)
            return fd, fsigC, fsigS
        elif typeBlur == 1:
            fksize = tb.Filter('KSIZE', 0, 100, 1)
            # twice because in trackbar module something bug if once, lol
            return fksize, fksize
        elif typeBlur == 2:
            fddepth = tb.Filter('DDEPTH', 0, 8, -1)
            fksize = tb.Filter('KSIZE', 0, 50, 1)
            fnormalize = tb.Filter('NORMOLIZE', 0, 1, 0)
            return fddepth, fksize, fnormalize
        elif typeBlur == 3:
            fshape = tb.Filter('SHAPE_TYPE', 0, 2, 0)
            fk = tb.Filter('KSIZE', 0, 50, 1)
            fits = tb.Filter('ITERS', 0, 50, 0)
            return fshape, fk, fits
        elif typeBlur == 4:
            fshape = tb.Filter('SHAPE_TYPE', 0, 2, 0)
            fk = tb.Filter('KSIZE', 0, 50, 1)
            fits = tb.Filter('ITERS', 0, 50, 0)
            return fshape, fk, fits
        elif typeBlur == 5:
            fshape = tb.Filter('SHAPE_TYPE', 0, 2, 0)
            fk = tb.Filter('KSIZE', 0, 50, 1)
            fddepth = tb.Filter('DDEPTH', 0, 50, 0)
            return fshape, fk, fddepth
        elif typeBlur == 6:
            fksize = tb.Filter('KSIZE', 0, 100, 1)
            fsx = tb.Filter('SIGMAX', 0, 300, 0)
            fsy = tb.Filter('SIGMAY', 0, 300, 0)
            return fksize, fsx, fsy
        elif typeBlur == 7:
            fksize = tb.Filter('KSIZE', 0, 100, 1)
            return fksize, fksize
        elif typeBlur == 8:
            fddepth = tb.Filter('DDEPTH', 0, 8, -1)
            fk = tb.Filter('KSIZE', 0, 7, 1)
            fsc = tb.Filter('SCALE', 0, 20, 1)
            fdt = tb.Filter('DELTA', 0, 300, 0)
            return fddepth, fk, fsc, fdt,

    def filteringImage(self, image, typeBlur, tbs):
        if typeBlur == 0:
            """d=-1 for compute itself; d=5 for online; d=9 for offline"""
            d, sigC, sigS, btType = tbs.get_trackbar_values()
            bt = self.getBorderType(btType)
            if imutils.is_cv2():
                if bt == 5:
                    bt = 4
            image = cv2.bilateralFilter(image, d, sigC, sigS, borderType=bt)
        elif typeBlur == 1:
            k, k, btType = tbs.get_trackbar_values()
            ksize = (k, k)
            bt = self.getBorderType(btType)
            image = cv2.blur(image, ksize, borderType=bt)
        elif typeBlur == 2:
            ddepth, k, n, btType = tbs.get_trackbar_values()
            ksize = (k, k)
            bt = self.getBorderType(btType)
            image = cv2.boxFilter(image, ddepth, ksize, normalize=n, borderType=bt)
        elif typeBlur == 3:
            shape, k, its, btType = tbs.get_trackbar_values()
            ksize = (k, k)
            bt = self.getBorderType(btType)
            if shape == 0:
                morph = cv2.MORPH_RECT
            elif shape == 1:
                morph = cv2.MORPH_CROSS
            else:
                morph = cv2.MORPH_ELLIPSE
            kernel = cv2.getStructuringElement(morph, ksize)
            image = cv2.dilate(image, kernel, iterations=its, borderType=bt)
        elif typeBlur == 4:
            shape, k, its, btType = tbs.get_trackbar_values()
            ksize = (k, k)
            bt = self.getBorderType(btType)
            if shape == 0:
                morph = cv2.MORPH_RECT
            elif shape == 1:
                morph = cv2.MORPH_CROSS
            else:
                morph = cv2.MORPH_ELLIPSE
            kernel = cv2.getStructuringElement(morph, ksize)
            image = cv2.erode(image, kernel, iterations=its, borderType=bt)
        elif typeBlur == 5:
            shape, k, ddepth, btType = tbs.get_trackbar_values()
            ksize = (k, k)
            bt = self.getBorderType(btType)
            if shape == 0:
                morph = cv2.MORPH_RECT
            elif shape == 1:
                morph = cv2.MORPH_CROSS
            else:
                morph = cv2.MORPH_ELLIPSE
            kernel = cv2.getStructuringElement(morph, ksize)
            image = cv2.filter2D(image, ddepth, kernel, delta=0)
        elif typeBlur == 6:
            k, sX, sY, btType = tbs.get_trackbar_values()
            if k % 2 == 1:
                ksize = (k, k)
            else:
                ksize = (k-1, k-1)
            bt = self.getBorderType(btType)
            image = cv2.GaussianBlur(image, ksize, sigmaX=sX, sigmaY=sY, borderType=bt)
        elif typeBlur == 7:
            k, k, btType = tbs.get_trackbar_values()
            if k % 2 == 0:
                k -= 1
            image = cv2.medianBlur(image, k)
        elif typeBlur == 8:
            ddepth, k, sc, dt, btType = tbs.get_trackbar_values()
            if k % 2 == 0:
                k -= 1
            ksize = cv2.getDerivKernels(2, 2, ksize=k)
            bt = self.getBorderType(btType)
            image = cv2.Laplacian(
                image, ddepth, ksize=k, scale=sc, delta=dt, borderType=bt)
        return image

    def callback(self, data, x):
        img = scripts.cv_methods_factory.exmodules.utils.getCVImage(data)
        if type(img[0, 0]) is not list:
            w, h, _ = img.shape
            #img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            w, h = img.shape
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        scale = 2
        img = cv2.resize(img, (h / scale, w / scale))
        self.queueRefresh.append('refresh')
        try:
            img = self.filteringImage(img, self.typeBlur, self.tbs)

            filterName = self.getFilterName(self.typeBlur)
            values = self.tbs.get_trackbar_values()
            rospy.loginfo(filterName)
            rospy.loginfo(values)
        except:
            rospy.loginfo('problems in a type border select! quickly move the slider!!!')
        msg = scripts.cv_methods_factory.exmodules.utils.getMsgImage(img)
        self.pubImage.publish(msg)

