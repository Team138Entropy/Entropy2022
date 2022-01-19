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

    with open('/boot/frc.json') as f:
        cameraConfig = json.load(f)
        #print(cameraConfig)
        camera = cameraConfig['cameras'][0]

    width = camera['width']
    height = camera['height']
    
    cs = CameraServer.getInstance()
    cameraSettings = cs.startAutomaticCapture()
    cameraConfig['pixel format'] = 'yuyv'
    cameraSettings.setConfigJson(json.dumps(cameraConfig))

    input_stream = cs.getVideo()
    output_stream = cs.putVideo('Processed', width, height)
    
    SocketThread = SocketWorker(PacketQueue).start()
    imgForm = np.zeros(shape=(height, width, 3), dtype=np.uint8)

    #hue = [0, 22]
    #sat = [110, 255]
    #val = [53, 255]  

    #Yellow Ball params
    yelHue = [18,49]
    yelSat = [52,255]
    yelVal = [166,255]

    kernel = np.ones((5,5),np.float32)/25
    radius = 5.855855855855857
    ksize = (2 * round(radius) + 1)

    solidLow = 70
    solidHigh = 100
    max_vertices = 50

    cy = ''
    cx = ''
    solid = 0

    print('Yellowball vision setup complete')

    while True:
        #Create info for packet
        PacketValue = {}
        PacketValue['cameraid'] = 0
        PacketValue['ballColor'] = 'yellow'
        
        start_time = time.time() #Use this to get FPS below
        frame_time, input_img = input_stream.grabFrame(imgForm)

        # Notify output of error and skip iteration
        if frame_time == 0:
            output_stream.notifyError(input_stream.getError())
            continue

        input_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)
        input_img = cv2.blur(input_img, (ksize, ksize))

        mask = cv2.inRange(input_img, (yelHue[0], yelSat[0], yelVal[0]),
                            (yelHue[1], yelSat[1], yelVal[1]))

        res = cv2.bitwise_and(input_img,input_img,mask = mask)
        res = cv2.cvtColor(res, cv2.COLOR_HSV2BGR)
        #cv2.imwrite('masked.jpg', res)

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

            if cntArea != 0 and hullArea != 0:
                solid = 100 * cntArea / hullArea

            validCnt = True 
            #print('Hullarea: ' , hullArea)
            validCnt &= (hullArea > 250) and (hullArea < 5000)
            #print('Perimeter:', perimeter)
            validCnt &= (perimeter > 50)
            #print('Width:', w)
            validCnt &= (w >= 25) and (w <= 350)
            #print('Height:', h)
            validCnt &= (h >= 15) and (h <= 300)
            #print('Solid:', solid)
            if solid != 0:
                validCnt &= (solid > solidLow) and (solid <= solidHigh)
            #print('Approximate Shape:', approximateShape)
            validCnt &= (len(approximateShape) >= 8)
            #print('Ratio:', ratio)
            validCnt &= (ratio >= 0) and (ratio < 5)
            
            #validCnt &= (y > cutOffHeight)

            validCnt &= (cv2.arcLength(cnt, True) < 10000)
            
            circularity = 0
            if(perimeter == 0):
                validCnt = False 
            else:
                circularity = 4*math.pi*(cntArea/(perimeter*perimeter))
                validCnt &= (.5 < circularity < 1.5)
            
            if(validCnt):
                #contSaveImage = cv2.drawContours(res, cnt, -1, (0, 255, 0), 3)
                #cv2.imwrite('contours.jpeg', contSaveImage)
                #print(cntArea, circularity, ratio)
                con.append(cnt)

        contimage = input_img

        # mask out rectange
        #cv2.rectangle(contimage, (0, 0), (width, int(cutOffHeight)), (0, 0, 0), -1)



        conCount = 0
        for cnt in con:
            conCount = conCount + 1
            x, y, w, h = cv2.boundingRect(cnt)
            
            M = cv2.moments(cnt)
            cy = int(M["m01"] / M["m00"])
            cx = int(M["m10"] / M["m00"])

            print('X center:', cx, 'Y center:',cy)

        PacketValue['BallX'] = cy
        PacketValue['BallY'] = cx
        #print(PacketValue)
        PacketQueue.put_nowait(PacketValue)
        cx = ''
        cy = ''

        #Calculates FPS part 2
        #processing_time = time.time() - start_time
        #fps = 1 / processing_time
        #print('FPS:' , fps)

        #Used to output image with contours drawn on it        
        #cv2.imwrite('testImg.jpeg', contimage)
