import json
import math
import queue
import socket
import sys
import threading
import time

import cv2
import numpy as np
from cscore import CameraServer
from networktables import NetworkTables


if __name__ == "__main__":
    with open('/boot/frc.json') as f:
        cameraConfig = json.load(f)
        camera = cameraConfig['cameras'][0]

    width = camera['width']
    height = camera['height']
    

    cs = CameraServer.getInstance()
    cameraSettings = cs.startAutomaticCapture()
    cameraConfig['pixel format'] = 'yuyv'
    cameraSettings.setConfigJson(json.dumps(cameraConfig))

    input_stream = cs.getVideo(cameraSettings = cameraSettings)
    output_stream = cs.putVideo('Processed', width, height)

    img = np.zeros(shape=(height, width, 3), dtype=np.uint8)

    #Yellow Ball params
    yelHue = [24,49]
    yelSat = [92,255]
    yelVal = [110,255]

    time.sleep(0.5)

    while True:
        start_time = time.time()
        frame_time, input_img = input_stream.grabFrame(img)

        output_img = np.copy(input_img)

        # Notify output of error and skip iteration
        if frame_time == 0:
            output_stream.notifyError(input_stream.getError())
            continue

        img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)