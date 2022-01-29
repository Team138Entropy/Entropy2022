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
    #cameraConfig['fps'] = '60'
    cameraSettings.setConfigJson(json.dumps(cameraConfig))

    input_stream = cs.getVideo()
    output_stream = cs.putVideo('Processed', res_width, res_height)
    
    SocketThread = SocketWorker(PacketQueue).start()

    #Numpy creates an array of zeros in the size of the image width/height. Its mentioned in documentation this can be performance intensive, and to do it outside the loop
    imgForm = np.zeros(shape=(res_height, res_width, 3), dtype=np.uint8)


    radius = 18
    ksize = (2 * round(radius) + 1)

    #Red Ball
    '''
    redHue = [0, 12]
    redSat = [109, 255]
    redVal = [58, 255]  
    '''

    #Blue ball
    blueHue = [86, 116]
    blueSat = [155, 255]
    blueVal = [0, 255]  

    #Yellow Ball params
    #yelHue = [18,49]
    #yelSat = [52,255]
    #yelVal = [166,255]

    current_frame = 0

    frame_time, input_img = input_stream.grabFrame(imgForm)

    # Notify output of error and skip iteration
    if frame_time == 0:
        output_stream.notifyError(input_stream.getError())

    cirlces = []

    print('Blue ball vision setup complete')

    print('1 second sleep...')
    time.sleep(1)
    print('Finished sleeping')

    while True:
        try:
            current_frame += 1
            PacketValue = {}
            PacketValue['cameraid'] = 0
            PacketValue['ballColor'] = 'yellow'
            
            #start_time = time.time() #Use this to get FPS below
            frame_time, input_img = input_stream.grabFrame(imgForm)

            # Notify output of error and skip iteration
            if frame_time == 0:
                output_stream.notifyError(input_stream.getError())
            
            '''
            print('Writing gray image')
            cv2.imwrite('Sanity.jpg', gray)
            '''

            mask = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)
            mask = cv2.blur(mask, (ksize, ksize))

            mask = cv2.inRange(mask, (blueHue[0], blueSat[0], blueVal[0]),
                                (blueHue[1], blueSat[1], blueVal[1]))


            
            #cimg = cv2.cvtColor(input_img, cv2.COLOR_BGR2GRAY)
                         
            cv2.imwrite('mask.jpg', mask)
            cimg = mask

            input_img_gray = cv2.cvtColor(input_img, cv2.COLOR_BGR2GRAY)
            cimg = cv2.bitwise_not(mask)  

            #cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
            
            #If you need to redo tuning, start param1 at 200 and work down, start param2 at 0 and work up.
            circles = cv2.HoughCircles(cimg, cv2.HOUGH_GRADIENT, 2, 100.0, param1=200, param2=20, minRadius = 10, maxRadius = 100)
            #circles = cv2.HoughCircles(cimg, cv2.HOUGH_GRADIENT, 1.2, 100)
            circles = np.uint16(np.around(circles))
            

            for i in circles[0,:]:
                # draw the outer circle
                cv2.circle(input_img,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(input_img,(i[0],i[1]),2,(0,0,255),3)

                print('x: ', i[0], print('y: '), i[1])
            
            
            cv2.imwrite('Hough.jpg', input_img)
            print('Writing hough circles...')
            
            '''
            input_img = cv2.blur(input_img, (ksize, ksize))

            mask = cv2.inRange(input_img, (blueHue[0], blueSat[0], blueVal[0]),
                                (blueHue[1], blueSat[1], blueVal[1]))
            '''
                
        except Exception as e:
            print('Error: ', e)

        '''
        print('Done, 1 sec sleep')
        time.sleep(1)
        '''