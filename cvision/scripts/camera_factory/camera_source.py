#!/usr/bin/env python

SOURCE_USBCAM = 'usb_cam'
SOURCE_CVCAMERA = 'cv_camera'
SOURCE_OPENNI2 = 'openni2'

TOPIC_USB_CAM_IMAGE_RAW = '/usb_cam/image_raw'
TOPIC_CV_CAMERA_IMAGE_RAW = '/cvcamera/image_raw'
TOPIC_OPENNI2_IMAGE_RAW = '/camera/rgb/image_raw'


class CameraSource:
    def getTopic(self):
        pass


class USBCAM(CameraSource):
    def getTopic(self):
        return TOPIC_USB_CAM_IMAGE_RAW


class CvCamera(CameraSource):
    def getTopic(self):
        return TOPIC_CV_CAMERA_IMAGE_RAW


class OpenNI2(CameraSource):
    def getTopic(self):
        return TOPIC_OPENNI2_IMAGE_RAW


class CameraFactory(object):
    """
    sourceType may be in ['usb_cam', 'cv_camera', 'openni2']
    :returns raw image topics
    """
    __instance = None

    def __new__(cls, val):
        if CameraFactory.__instance is None:
            CameraFactory.__instance = object.__new__(cls)
        CameraFactory.__instance.val = val
        return CameraFactory.__instance

    def getCameraSource(self, sourceType):
        if not sourceType:
            raise IOError
        if sourceType == SOURCE_USBCAM:
            return USBCAM()
        elif sourceType == SOURCE_CVCAMERA:
            return CvCamera()
        elif sourceType == SOURCE_OPENNI2:
            return OpenNI2()
