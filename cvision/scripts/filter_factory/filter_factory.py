import yaml


class Values:
	def getValues(self):
		return self.getValues

class InitValues(Values):
	INIT_TRACKBAR_FILE_NAME = 'trackbars.yaml'
	INIT_FILTER_FILE_NAME = 'filters.yaml'
	def getValues(self):
		pass

class FilterInitValues(Values):
	def getValues(filterTitle):
		try:
			# min, max, value
			file = open(InitValues.INIT_FILTER_FILE_NAME, 'r')
			if file:
				raw = file.read()
				data = yaml.load(raw)

				return(values)
		except:
			print('config file failt!')


class TrackbarInitValues(InitValues):
	def getValues(self, fieldName):
		try:
			# min, max, value
			file = open(InitValues.INIT_TRACKBAR_FILE_NAME, 'r')
			if file:
				values = file.read()
				return value
		except:
			return print('init file failt!')


# trackbars
class Trackbar:
	def __init__(self, init=(title, min, max, value)):
		self.title = title
		self.min = min
		self.max = max
		self. value = value
	def callback(self, value):
		self.value = value
		
class TrackbarFactory:
	def getTrackbar(self, fieldName):
		if fieldName in ['ksize', 'threshold1', 'threshold2']
			initValues = TrackbarInitValues.getValues(fieldName)
			return Trackbar(fieldName, initValues)
		return -2
	getTrackbar = staticmethod(getTrackbar)

class TrackbarWindow:
	def __init__(self, winTitle)
		self.winTitle = winTitle
		self.trackbars = []
	def addTrackbar(self, trackbar):
		cv2.createTrackbar(trackbar.title, self.winTitle, trackbar.value, trackbar.max, trackbar.callback)
		self.trackbar.append(trackbar)
	def getTrackbarValue(self, trackbarTitle):
		pass
	def getTrackbarVlues(self):
		pass

class Fields: # FieldsFactory
	def getFields(self):
		return self.getFields()

class FilterFields(Fields): # FilterFieldsFactory
	KERNEL_SIZE = 'ksize'
	THRESHOLD_1 = 'threshold1'
	THRESHOLD_2 = 'threshold2'

	BLUR_FIELDS = [KERNEL_SIZE]
	CANNY_FIELDS = [THRESHOLD_1, THRESHOLD_2]

	def getFields(self, filterTitle):
		return = {
			Filter.BLUR_TITLE: BLUR_FIELDS
			Filter.CANNY_TITLE: CANNY_FIELDS
		}[filterTitle]
	getFields = staricmethod(getFields)

class Parameters:
	def getParameters(self):
		pass


class FilterParameters(Parameters):
	def getParameters(self, filterTitle, mode):
		if mode: # configur by trackbars
			# MUST UPDATE IN REAL TIME FOR
			# should run in thread
			flrFilds = FilterFields.getFields(filterTitle)
			tbrWin = TrackbarWindow(filterTitle)
			tbrs = []
			for field in flrFilds:
				initValues = TrackbarInitValues.getValues(field)
				tbr = Trackbar(field, initValues)
				tbrWin.addTrackbar(tbr)
				params = tbrWin.getTrackbarVlues()
		else: # configure by config file
			params = FilterInitValues.getValues(filterTitle)
		return params

	getParameters = staticmethod(getParameters)





# filters
class Filter:
	BLUR_TITLE = 'blur'
	CANNY_TITLE = 'canny'

	def apply(self):
		return self.apply()

class Blur(Filter):
	def __init__(self, params):
		self.ksize = params.ksize
	def apply(self):
		return 'Blur is applyed!'

class Canny(Filter):
	def __init__(self, params):
		self.th1 = params.th1
		self.th2 = params.th2
	def apply(self):
		return 'Canny is applyed!'


class FilterFactory:
	"""mode=1 is Trackbars; mode=0 is readFromFile"""
	__fp = FilterParameters()
	def getFilter(self, filterTitle, mode=1):
		if filterTitle in ['blur', 'canny']:
			params = __fp.getParameters(filterTitle, mode)
			if filterTitle == Filter.BLUR_TITLE:
				return Blur(params)
			elif filterTitle == Filter.CANNY_TITLE:
				return Canny(params)
		return -1
	getFilter = staticmethod(getFilter)


