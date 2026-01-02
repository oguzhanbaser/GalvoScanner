from picamera2 import Picamera2
import cv2
import time
import os

class MyCamera:
    def __init__(self):
        # Kameranın çözünürlüğünü ortam değişkenlerinden al (varsayılan 640x480)

        # CAMERA_WIDTH = 3280
        # CAMERA_HEIGHT = 2464

        self.CAMERA_WIDTH = 2592
        self.CAMERA_HEIGHT = 1944
        self.CAMERA_FOV_X = 102
        self.CAMERA_FOV_Y = 66
        self.ROI_FOV_X = 45
        self.ROI_FOV_Y = 30

        # Kamerayı başlat
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"size": (self.CAMERA_WIDTH, self.CAMERA_HEIGHT), "format": "RGB888"})
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1)  # Kameranın oturması için kısa bekleme

    def calculate_roi(self):
        ROI_WIDTH = (self.CAMERA_WIDTH * self.ROI_FOV_X) // self.CAMERA_FOV_X
        ROI_HEIGHT = (self.CAMERA_HEIGHT * self.ROI_FOV_Y) // self.CAMERA_FOV_Y
        ROI_X = (self.CAMERA_WIDTH - ROI_WIDTH) // 2
        ROI_Y = (self.CAMERA_HEIGHT - ROI_HEIGHT) // 2
        return ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT

    def get_frame(self):
        # Kameradan bir frame al
        frame = self.picam2.capture_array()
        return frame

    def get_frame_roi(self):
        my_roi = self.calculate_roi()
        frame = self.picam2.capture_array()
        frame_roi = frame[ my_roi[1]:my_roi[1] + my_roi[3], my_roi[0]:my_roi[0] + my_roi[2]]
        return frame_roi



