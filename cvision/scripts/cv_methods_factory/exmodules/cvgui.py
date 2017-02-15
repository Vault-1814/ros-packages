import cv2


class Trackbar:
    def __init__(self, title, (min, max, value)):
        self.title = title
        self.min = min
        self.max = max
        self.value = value


class TrackbarWindow:
    def __init__(self, win_name, *filters):
        self.win_name = win_name
        self.all_filters = []
        self.add_filters(filters)
        self.setup_trackbars(self.all_filters)
        self.isChange = False

    def set_win_name(self, win_name):
        self.win_name = win_name

    def get_win_name(self):
        return self.win_name

    def add_filter(self, filter):
        self.all_filters.append(filter)
        self.setup_trackbars(self.all_filters)

    def add_filters(self, *filters):
        if len(filters) == 1:
            filters = filters[0]
        for f in filters:
            print(f.title + ' filter was added!')
            self.all_filters.append(f)
        self.setup_trackbars(self.all_filters)

    def callback(self, value):
        self.isChange = True

    def isChanged(self):
        if self.isChange:
            self.isChange = False
            return True
        return False

    def setup_trackbars(self, range_filter):
        cv2.namedWindow(self.win_name, 0)
        for i in range_filter:
            cv2.createTrackbar(str(i.title), self.win_name, int(i.value), i.max, self.callback)

    def get_trackbar_values(self):
        values = []
        for i in self.all_filters:
            v = cv2.getTrackbarPos(str(i.title), self.win_name)
            values.append(v)
        return values

    def get_trackbar_value(self, filter_title):
        for f in self.all_filters:
            if f.title == filter_title:
                return cv2.getTrackbarPos(str(f.title), self.win_name)
