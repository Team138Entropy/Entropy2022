from cscore import CameraServer
import cv2
import numpy as np
import os
import re
import json

if __name__ == "__main__":
    print("Vision capture starting")

    directory = "samples"

    if not os.path.isdir(directory):
        print("Directory doesn't exist, creating now")
        os.makedirs(directory)

    files = os.listdir("./" + directory)
    print("Files in directory: ", files)

    if len(files) > 1:
        series = re.search("(?<=_)\d*(?=_)", files[len(files) - 1]).string
    else:
        series = 1
    
    print("The current series is: ", series)

    with open('/boot/frc.json') as f:
        cameraConfig = json.load(f)
        print("Camera config is: ", cameraConfig)
        camera = cameraConfig['cameras'][0]

    res_width = camera['width']
    res_height = camera['height']
    
    #Start camera server, start capturing from the camera and set the pixel format
    cs = CameraServer.getInstance()
    cameraSettings = cs.startAutomaticCapture()
    cameraConfig['pixel format'] = 'yuyv'
    cameraConfig['FPS'] = '60'
    cameraSettings.setConfigJson(json.dumps(cameraConfig))

    print('Set to 60 now')

    input_stream = cs.getVideo()
    output_stream = cs.putVideo('Processed', res_width, res_height)

    #Numpy creates an array of zeros in the size of the image width/height. Its mentioned in documentation this can be performance intensive, and to do it outside the loop
    imgForm = np.zeros(shape=(res_height, res_width, 3), dtype=np.uint8)

    while True:
        time, input_img = input_stream.grabFrame(imgForm)

        if time == 0:
            continue

        cv2.imwrite("./" + directory + "/img_" + str(series) + "_" + str(time) + ".jpg", input_img)
        print("./" + directory + "/img_" + str(series) + "_" + str(time) + ".jpg")

        
