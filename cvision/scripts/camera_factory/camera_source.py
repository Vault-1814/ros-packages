#!/usr/bin/env python
import config


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


class CameraFactory:
    """
    sourceType may be in ['usb_cam', 'cv_camera', 'openni2']
    :returns raw image topics
    """
    def getCameraSource(self, sourceType):
        if not sourceType:
            raise IOError
        if sourceType == config.SOURCE_USBCAM:
            return USBCAM()
        elif sourceType == config.SOURCE_CVCAMERA:
            return CvCamera()
        elif sourceType == config.SOURCE_OPENNI2:
            return OpenNI2()
