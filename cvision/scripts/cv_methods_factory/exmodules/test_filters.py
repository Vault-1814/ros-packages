import rospy
import cv2
import imutils
import cvgui
import utils
import threading
from collections import deque
import time
from sensor_msgs.msg import Image
import cvfeatures.filter_factory as ff

FILTER_SELECTOR = 'FILTER'


class RefreshTrackbarsThread(threading.Thread):
    def __init__(self, queue, cfgFilteringNode):
        threading.Thread.__init__(self)
        self.queue = queue
        self.cfn = cfgFilteringNode

    def run(self):
        while True:
            while len(self.queue) == 0:
                time.sleep(0.1)
            #typeFilter = self.queue[0]  # int value
            if self.cfn.typeFilter != self.cfn.oldTypeFilter:
                rospy.loginfo('new filter selected!')
                # close the window for old filter
                if self.cfn.tbsWindow is not None:
                    tbsWindowName = self.cfn.tbsWindow.get_win_name()
                    cv2.destroyWindow(tbsWindowName)
                # open the window for new filter
                filterName = ff.getFilterName(self.cfn.typeFilter)
                self.cfn.tbsWindow = cvgui.TrackbarWindow(filterName)
                rospy.loginfo(self.cfn.tbsWindow.get_win_name())
                self.cfn.filter = self.cfn.flrf.getFilter(filterName, self.cfn.tbsWindow)
                self.cfn.oldTypeFilter = self.cfn.typeFilter
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
        self.tbsWindow = None
        self.filter = None
        self.flrf = ff.FilterFactory()

        # create a trackbar for selection different filters
        self.ftypeFilter = cvgui.Trackbar(FILTER_SELECTOR, (0, 4, 0))
        self.tbsFilterSelector = cvgui.TrackbarWindow('filter selector',
                                                      self.ftypeFilter)
        self.typeFilter = 0
        self.oldTypeFilter = -1

        self.subCamera = rospy.Subscriber(
            cameraTopic, Image, self.callback)
        self.pubImage = rospy.Publisher(
            dstTopic, Image, queue_size=1)

        self.queueRefresh = deque([], 1)
        self.displayThread = RefreshTrackbarsThread(self.queueRefresh, self)
        self.displayThread.setDaemon(True)
        self.displayThread.start()

    def callback(self, date):
        image = utils.getCVImage(date)
        self.typeFilter = self.tbsFilterSelector.get_trackbar_value(FILTER_SELECTOR)
        self.queueRefresh.append(0)
        image = self.filter.filtering(image)
        try:
            pass
        except:
            rospy.loginfo('problem with filtering!')
        msg = utils.getMsgImage(image)
        self.pubImage.publish(msg)
