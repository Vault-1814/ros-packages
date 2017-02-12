#!/usr/bin/env python
import config


class CameraSource:
    def publish(self):
        pass


class USBCAM(CameraSource):
    def publish(self):
        return config.TOPIC_USB_CAM_IMAGE_RAW


class CvCamera(CameraSource):
    def publish(self):
        return config.TOPIC_CV_CAMERA_IMAGE_RAW


class OpenNI2(CameraSource):
    def publish(self):
        return config.TOPIC_OPENNI2_IMAGE_RAW


class CameraFactory:
    """
    sourceType may be in ['usb_cam', 'cv_camera', 'openni2']
    :returns raw image topics
    """
    def get_camera_source(self, sourceType):
        if not sourceType:
            raise IOError
        if sourceType == config.SOURCE_USBCAM:
            return USBCAM()
        elif sourceType == config.SOURCE_CVCAMERA:
            return CvCamera()
        elif sourceType == config.SOURCE_OpenNI2:
            return OpenNI2()
