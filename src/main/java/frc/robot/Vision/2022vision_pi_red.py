import json
import math
import queue
import socket
import threading

import cv2
import numpy as np
from cscore import CameraServer
from networktables import NetworkTables

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
    cameraSettings.setConfigJson(json.dumps(cameraConfig))

    input_stream = cs.getVideo()
    output_stream = cs.putVideo('Processed', res_width, res_height)
    
    SocketThread = SocketWorker(PacketQueue).start()

    #Numpy creates an array of zeros in the size of the image width/height. Its mentioned in documentation this can be performance intensive, and to do it outside the loop
    imgForm = np.zeros(shape=(res_height, res_width, 3), dtype=np.uint8)

    #Red Ball
    redHue = [0, 12]
    redSat = [109, 255]
    redVal = [58, 255]  

    #Blue ball estimates
    blueHue = [78, 120]
    blueSat = [156, 255]
    blueVal = [28, 255]  

    #Yellow Ball params
    #yelHue = [18,49]
    #yelSat = [52,255]
    #yelVal = [166,255]

    #Creating settings for blur filter
    radius = 5.855855855855857
    ksize = (2 * round(radius) + 1)

    #Parameters for targeting, I set these all up here because its easier to go through and change them when tuning with grip
    hull_area_low = 3000
    hull_area_high = 7000
    minimum_perimeter = 75
    width_minimum = 100
    width_maximum = 300
    height_minimum = 30
    height_maximum = 300
    solid_Low = 94
    solid_High = 100
    max_vertices = 100
    rat_low = 0
    rat_high = 10
    cy = ''
    cx = ''
    solid = 0
    last_cnt_area = 0
    conCount = 0
    cnt_to_process = 0
    lowest_y = 1000000

    #Create info for packet
    PacketValue = {}
    print('Yellowball vision setup complete')

    while True:
        #Create info for packet
        try:
            PacketValue = {}
            PacketValue['cameraid'] = 0
            PacketValue['ballColor'] = 'yellow'
            
            #start_time = time.time() #Use this to get FPS below
            frame_time, input_img = input_stream.grabFrame(imgForm)

            # Notify output of error and skip iteration
            if frame_time == 0:
                output_stream.notifyError(input_stream.getError())
                continue

            input_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)
            input_img = cv2.blur(input_img, (ksize, ksize))

            mask = cv2.inRange(input_img, (blueHue[0], blueSat[0], blueVal[0]),
                                (blueHue[1], blueSat[1], blueVal[1]))

            _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)

            # Sort contours by area size (biggest to smallest)
            cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

            con = []
            for cnt in cntsSorted:
                # Get moments of contour; mainly for centroid
                M = cv2.moments(cnt)
                # Get convex hull (bounding polygon on contour)
                hull = cv2.convexHull(cnt)
                # Calculate Contour area
                cntArea = cv2.contourArea(cnt)
                # calculate area of convex hull
                hullArea = cv2.contourArea(hull)
                    
                # Approximate shape
                approximateShape = cv2.approxPolyDP(hull, 0.01 * cv2.arcLength(hull, True), True)

                x, y, w, h = cv2.boundingRect(hull)
                ratio = float(w) / h

                perimeter = cv2.arcLength(cnt, True)
                

                # finding center point of shape
                M = cv2.moments(hull)
                if M['m00'] != 0.0:
                    x = int(M['m10']/M['m00'])
                    y = int(M['m01']/M['m00'])

                #Frequently attempts to divide by zero, so a check that they aren't is necessary
                if cntArea != 0 and hullArea != 0:
                    solid = 100 * cntArea / hullArea

                #Filtering out the contours based on tuned values we are looking for
                validCnt = True 
                validCnt &= (hullArea > hull_area_low)# and (hullArea < hull_area_high)
                validCnt &= (perimeter > minimum_perimeter)
                validCnt &= (w >= width_minimum) and (w <= width_maximum)
                validCnt &= (h >= height_minimum) and (h <= height_maximum)
                if solid != 0:
                    validCnt &= (solid > solid_Low) and (solid <= solid_High)
                validCnt &= (len(approximateShape) >= 8)
                validCnt &= (ratio >= rat_low) and (ratio < rat_high)

                '''
                print('Hullarea: ' , hullArea)
                print('Perimeter:', perimeter)
                print('Width:', w)
                print('Height:', h)
                print('Solid:', solid)
                print('Approximate Shape:', approximateShape)
                print('Ratio:', ratio)
                '''
                
                #validCnt &= (y > cutOffHeight)

                validCnt &= (cv2.arcLength(cnt, True) < 10000)
                
                circularity = 0
                if(perimeter == 0):
                    validCnt = False 
                else:
                    circularity = 4*math.pi*(cntArea/(perimeter*perimeter))
                    validCnt &= (.5 < circularity < 1.5)
                
                if(validCnt) and y < lowest_y :
                    #print(cntArea, circularity, ratio)
                    cnt_to_process = cnt

            x, y, w, h = cv2.boundingRect(cnt_to_process)
            
            M = cv2.moments(cnt_to_process)
            cy = int(M["m01"] / M["m00"])
            cx = int(M["m10"] / M["m00"])

            print('X center:', cx, 'Y center:',cy)

            PacketValue['BallX'] = cy
            PacketValue['BallY'] = cx
            #print(PacketValue)
            PacketQueue.put_nowait(PacketValue)
            cx = ''
            cy = ''
            last_cnt_area = 0
            cnt_to_process = ''
            lowest_y = 1000000
        except:
            print('Error, likely that a ball wasnt found')

