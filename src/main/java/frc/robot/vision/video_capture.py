from cscore import CameraServer
import cv2
import numpy as np
import os
import re
import json
import time

if __name__ == "__main__":
    print("Vision capture program starting")

    print("Setting directory to save capture to")
    directory = "./video_samples"

    print('Creating video sample directory if it does not exist')
    if not os.path.isdir(directory):
        print("Directory doesn't exist, creating now")
        os.makedirs(directory)

    #Opens the JSON config for camera settings, but in this case I think we can cut it out
    
    print('Opening JSON config')
    with open('/boot/frc.json') as f:
        cameraConfig = json.load(f)
        print("Camera config is: ", cameraConfig)
        camera = cameraConfig['cameras'][0]

    print('Setting camera width height')
    res_width = camera['width']
    res_height = camera['height']
    
    #Start camera server, start capturing from the camera and set the pixel format
    print('Creating camera server')
    cs = CameraServer.getInstance()
    cameraSettings = cs.startAutomaticCapture()
    cameraConfig['pixel format'] = 'mjpeg'
    cameraConfig['FPS'] = '60'
    print('FPS set to 60 now')
    #cameraSettings.setConfigJson(json.dumps(cameraConfig))

    input_stream = cs.getVideo()
    output_stream = cs.putVideo('Processed', res_width, res_height)

    #TODO Capture the .mp4 stream here
    #Consider how we will save the video despite not knowing when the robot will be powered off.
    cap = 1
    errorValue = 0
    #time.sleep(10)
    try:
        print("using output stream")
        cap = cv2.VideoCapture(output_stream)
    except Exception as e:
        print(str(e))
    '''
    while True:
        try:     
            print("using URL")
            cap = cv2.VideoCapture('http://wpilibpi.local:1181/stream.mjpg')
            print("cap:", cap)
        except Exception as e:
            print(str(e))
    '''
    print("it worked!")

    '''
    #Example of just constantly capturing images
    #Numpy creates an array of zeros in the size of the image width/height. Its mentioned in documentation this can be performance intensive, and to do it outside the loop
    imgForm = np.zeros(shape=(res_height, res_width, 3), dtype=np.uint8)

    while True:
        time, input_img = input_stream.grabFrame(imgForm)

        if time == 0:
            continue

        cv2.imwrite(directory + "/img_" + str(series) + "_" + str(time) + ".jpg", input_img)
        print(directory + "/img_" + str(series) + "_" + str(time) + ".jpg")
    '''