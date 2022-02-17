import json
import math
import queue
import socket
import threading
import time
from sqlite3 import Time

import cv2
from cv2 import imwrite
import numpy as np
from cscore import CameraServer

#Below link has a barebones version, useful for getting the camera server stuff
#https://docs.wpilib.org/en/stable/docs/software/vision-processing/wpilibpi/basic-vision-example.html

# Queue of Packets
# Thread Safe.. Packets being sent to robot are placed here!
PacketQueue = queue.Queue()

# Creates a socket
#No clue if this will work, just pulled it from last year
class SocketWorker(threading.Thread):
    def __init__(self, q, *args, **kwargs):
        self.queue = q
        super().__init__(*args, **kwargs)

        # Initialize Socket Connect
        SocketHost = "10.1.38.2"
        SocketPort = 5800
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.connect((SocketHost, SocketPort))

    def run(self):
        while True:
            try:
                packet = self.queue.get()
                # Convert Packet to JSON
                # Send Message Over Socket
                try:
                    data_string = json.dumps(packet)
                    self.sock.sendall(data_string.encode())
                except Exception as e:
                    print("Socket Exception " + str(e))
            except Exception as e1:
                pass

if __name__ == "__main__":
    #Avoid touching camera server settings
    print('2022 Ball Vision Yellow Starting')

    #Load camera config (eg. Exposure, resolution, FPS)
    with open('/boot/frc.json') as f:
        cameraConfig = json.load(f)
        #print(cameraConfig)
        camera = cameraConfig['cameras'][0]

    res_width = camera['width']
    res_height = camera['height']
    
    #Start camera server, start capturing from the camera and set the pixel format
    cs = CameraServer.getInstance()
    cameraSettings = cs.startAutomaticCapture()
    cameraConfig['pixel format'] = 'yuyv'
    cameraConfig['fps'] = '60'
    cameraSettings.setConfigJson(json.dumps(cameraConfig))

    input_stream = cs.getVideo()
    output_stream = cs.putVideo('Processed', res_width, res_height)
    
    SocketThread = SocketWorker(PacketQueue).start()

    #Numpy creates an array of zeros in the size of the image width/height. Its mentioned in documentation this can be performance intensive, and to do it outside the loop
    imgForm = np.zeros(shape=(res_height, res_width, 3), dtype=np.uint8)

    #Red Ball
    redHue = [0, 17]
    redSat = [161, 255]
    redVal = [37, 255]  

    #Blue ball
    '''
    blueHue = [85, 112]
    blueSat = [161, 255]
    blueVal = [37, 255]  
    '''

    #Yellow Ball params
    #yelHue = [18,49]
    #yelSat = [52,255]
    #yelVal = [166,255]

    #Creating settings for blur filter
    radius = 5.5
    ksize = (2 * round(radius) + 1)

    '''
    #Parameters for targeting, I set these all up here because its easier to go through and change them when tuning with grip
    #Parameters for targeting, I set these all up here because its easier to go through and change them when tuning with grip
    cnt_area_low = 4000
    #cnt_area_high = 7500
    minimum_perimeter = 70
    width_minimum = 50
    width_maximum = 300
    height_minimum = 30
    height_maximum = 300
    solid_Low = 94
    solid_High = 100
    min_vertices = 20
    max_vertices = 100
    rat_low = 0
    rat_high = 3
    cy = ''
    cx = ''
    solid = 0
    last_cnt_area = 0
    conCount = 0
    cnt_to_process = 0
    '''

    #Used to pick the lowest detected contour in the image which is most likely the closest ball
    lowest_y = 1000

    last_contour_x = 1000
    last_contour_y = 1000
    current_frame = 0
    frame_memory = 0
    contour_found_once = False

    #Create info for packet
    PacketValue = {}
    
    dark_blobs = False
    min_area = 1.0
    circularity = [.7284172661870504, 1.0]

    params = cv2.SimpleBlobDetector_Params()
    params.filterByColor = 1
    params.blobColor = (0 if dark_blobs else 255)
    params.minThreshold = 10
    params.maxThreshold = 220
    params.filterByArea = True
    params.minArea = min_area
    params.filterByCircularity = True
    params.minCircularity = circularity[0]
    params.maxCircularity = circularity[1]
    params.filterByConvexity = False
    params.filterByInertia = False
    detector = cv2.SimpleBlobDetector_create(params)

    blank = np.zeros((1, 1))

    draw_contours = ''
    first_run = True
    res = ''
    keypoints = ''
    blob_x = ''
    blob_y = ''

    print('Blue ball blob vision setup complete')

    while True:
        #Create info for packet
        try:
            current_frame += 1
            PacketValue = {}
            PacketValue['cameraid'] = 0
            PacketValue['ballColor'] = 'blue'
            
            #start_time = time.time() #Use this to get FPS below
            frame_time, input_img = input_stream.grabFrame(imgForm)

            # Notify output of error and skip iteration
            if frame_time == 0:
                output_stream.notifyError(input_stream.getError())
                continue

            input_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)
            input_img = cv2.blur(input_img, (ksize, ksize))

            mask = cv2.inRange(input_img, (redHue[0], redSat[0], redVal[0]),
                                (redHue[1], redSat[1], redVal[1]))
            cv2.imwrite('Mask.jpeg', mask)

            #inverted_img = cv2.bitwise_not(mask)
            
            try:
                keypoints = detector.detect(mask)
            except Exception as e:
                print('Detector exception: ', e)
            try:
                cv2.imwrite('Keypoints.jpeg', keypoints)
            except Exception as e:
                print('Keypoints exception: ', e)

            if keypoints:
                print('Has keypoints')
                try:
                    blob_x = keypoints[0].pt[0]
                    blob_y = keypoints[0].pt[1]
                except Exception as e:
                    print('Couldnt get x + y of keypoints: ', e)
                print('X: %s, Y: %s' % (blob_x, blob_y))
                blobs = cv2.drawKeypoints(mask, keypoints,np.array([]),(0, 0, 255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                cv2.imwrite('Blobs.jpeg', blobs)
                print('sleeping')
                time.sleep(2)

            keypoints = ''
            blob_x = ''
            blob_y = ''
            
            '''
            _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)

            # Sort contours by area size (biggest to smallest)
            cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
            '''
        except Exception as e:
            print('Exception', e)