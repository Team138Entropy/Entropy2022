import json
import logging
import math
import queue
import socket
import sys
import threading
import time
from multiprocessing.sharedctypes import Value
from sqlite3 import Time

import cv2
import numpy as np
from cscore import CameraServer
from networktables import NetworkTables, NetworkTablesInstance

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

def calculateDistanceFeet(targetPixelWidth):
    # d = Tft*FOVpixel/(2*Tpixel*tanΘ)
    
    #Target width in feet, ball for 2022 is ~9 1/2 inches (so tft is just an estimate to 9 inches)
    Tft = .75
    #170 is our cameras FOV for 2022. PSEye was 75
    tanFOV = math.tan(170/2)
    #For some reason, 640 works for a 320 resolution width, if set to 320 it may remove the need for the arbitrary value
    distEst = Tft * 640 / (2 * targetPixelWidth * tanFOV)
    # Unsure as to what measurement distEst is producing in the above line, but multiplying it by .32 will return your distance in feet
    distEstFeet = distEst * .32
    return (abs(distEstFeet))

if __name__ == "__main__":
    #Avoid touching camera server settings
    print('2022 Ball Vision Blue Starting')
    
    cameraConfig = ''
    with open('/boot/frc.json', 'r') as f:
        mycamera = json.load(f)
        cameraConfig = mycamera
        camera = cameraConfig['cameras'][0]
        '''
        Create this list by going to wpilibpi.local, vision settings, open stream, change settings to whats needed, open "source config JSON",
        then paste it into "Custom Properties Json" in Vision Settings, Save it, then copy the list created, and replace whatevers set in the cameraConfig variable
        Dont try to create this manually, its extremely picky on formatting.
        '''
        cameraConfig = {"fps":120,"height":240,"pixel format":"mjpeg","properties":[{"name":"connect_verbose","value":1},{"name":"raw_brightness","value":-8},{"name":"brightness","value":43},{"name":"raw_contrast","value":0},{"name":"contrast","value":0},{"name":"raw_saturation","value":128},{"name":"saturation","value":100},{"name":"raw_hue","value":0},{"name":"hue","value":50},{"name":"white_balance_temperature_auto","value":True},{"name":"gamma","value":100},{"name":"raw_gain","value":0},{"name":"gain","value":0},{"name":"power_line_frequency","value":1},{"name":"white_balance_temperature","value":4600},{"name":"raw_sharpness","value":2},{"name":"sharpness","value":33},{"name":"backlight_compensation","value":1},{"name":"exposure_auto","value":3},{"name":"raw_exposure_absolute","value":157},{"name":"exposure_absolute","value":3},{"name":"exposure_auto_priority","value":True}],"width":320}
        

    cs = CameraServer.getInstance()
    cameraSettings = cs.startAutomaticCapture()

    cameraSettings.setConfigJson(json.dumps(cameraConfig))
    input_stream = cs.getVideo()
    output_stream = cs.putVideo('Processed', 320, 240)

    SocketThread = SocketWorker(PacketQueue).start()

    #Numpy creates an array of zeros in the size of the image width/height. Its mentioned in documentation this can be performance intensive, and to do it outside the loop
    #Note that the order is (height, width) in shape
    imgForm = np.zeros(shape=(240, 320, 3), dtype=np.uint8)

    #Blue ball
    blueHue = [85, 122]
    blueSat = [119, 255]
    blueVal = [41, 255]  

    #Creating settings for blur filter. Radius should be figured out by testing in GRIP
    radius = 2.83
    ksize = (2 * round(radius) + 1)

    #Parameters for targeting, I set these all up here because its easier to go through and change them when tuning with grip
    cnt_area_low = 500
    #cnt_area_high = 7500
    minimum_perimeter = 10
    width_minimum = 10
    width_maximum = 300
    height_minimum = 10
    height_maximum = 300
    solid_Low = 94
    solid_High = 100
    min_vertices = 18
    max_vertices = 100
    rat_low = 0
    rat_high = 3
    cy = ''
    cx = ''
    solid = 0
    last_cnt_area = 0
    conCount = 0
    cnt_to_process = 0

    #Used to pick the lowest detected contour in the image which is most likely the closest ball
    lowest_y = 1000

    last_contour_x = 1000
    last_contour_y = 1000
    current_frame = 0

    #Create empty dictionary for values we'll send in our packet
    PacketValue = {}

    '''
    #Values used for testing framerate
    curTime = time.time()
    oldTime = ''
    '''
    printCount = 1
    myDistFeet = 0
 
    black_mask = cv2.imread('/home/pi/black_mask.png')
    
    print('Blue ball vision setup complete')

    while True:
        #Try covers the following code to make sure we never fail during a match.
        try:
            current_frame += 1
            '''
            if current_frame % 60 == 0:
                oldTime = curTime
                curTime = time.time()
                print(curTime - oldTime)
            '''

            PacketValue = {}
            PacketValue['cameraid'] = 0
            PacketValue['ballColor'] = 'blue'
            
            #start_time = time.time() #Use this to get FPS below
            frame_time, input_img = input_stream.grabFrame(imgForm)

            # Notify output of error and skip iteration
            if frame_time == 0:
                output_stream.notifyError(input_stream.getError())
                continue

            #Change inmage colorspace to HSV, blur it, and for the 2022 robot we flip the image due to the camera being upside down.
            input_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)
            input_img = cv2.blur(input_img, (ksize, ksize))
            input_img = cv2.flip(input_img, 1)

            #Mask out colors that dont fall in the range we'd find the blue ball in
            mask = cv2.inRange(input_img, (blueHue[0], blueSat[0], blueVal[0]),
                                (blueHue[1], blueSat[1], blueVal[1]))

            _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)

            # Sort contours by area size (biggest to smallest)
            cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

            con = []
            for cnt in cntsSorted:
                # Calculate Contour area early as a way to eliminate unnecesarily processing contours that arent viable
                cntArea = cv2.contourArea(cnt)
                if (cntArea > cnt_area_low):# and (cntArea < cnt_area_high)
                    # Get moments of contour; mainly for centroid
                    M = cv2.moments(cnt)
                    # Get convex hull (bounding polygon on contour)
                    hull = cv2.convexHull(cnt)
                    
                    # calculate area of convex hull
                    #hullArea = cv2.contourArea(hull)
                        
                    # Approximate shape - Approximates a polygonal curve(s) with the specified precision.
                    approximateShape = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)

                    x, y, w, h = cv2.boundingRect(cnt)
                    #Check what amount of the bounding rectangle is filled by the detected object.
                    ratio = float(w) / h

                    perimeter = cv2.arcLength(cnt, True)
                    

                    # finding center point of shape
                    M = cv2.moments(cnt)
                    if M['m00'] != 0.0:
                        x = int(M['m10']/M['m00'])
                        y = int(M['m01']/M['m00'])

                    #Frequently attempts to divide by zero, so a check that they aren't is necessary
                    '''
                    if cntArea != 0 and cntArea != 0:
                        solid = 100 * cntArea / hullArea
                    '''

                    #Filtering out the contours based on tuned values we are looking for
                    validCnt = True 
                    validCnt &= (perimeter > minimum_perimeter)
                    validCnt &= (w >= width_minimum) and (w <= width_maximum)
                    validCnt &= (h >= height_minimum) and (h <= height_maximum)
                    '''
                    if solid != 0:
                        validCnt &= (solid > solid_Low) and (solid <= solid_High)
                    '''
                    validCnt &= (len(approximateShape) >= 8)
                    validCnt &= (ratio >= rat_low) and (ratio < rat_high)

                    '''
                    #List of prints for debugging
                    print('Hullarea: ' , hullArea)
                    print('Perimeter:', perimeter)
                    print('Width:', w)
                    print('Height:', h)
                    print('Solid:', solid)
                    print('Approximate Shape:', approximateShape)
                    print('Ratio:', ratio)
                    '''

                    validCnt &= (cv2.arcLength(cnt, True) < 10000)
                    
                    circularity = 0
                    if(perimeter == 0):
                        validCnt = False 
                    else:
                        circularity = 4*math.pi*(cntArea/(perimeter*perimeter))
                        validCnt &= (.5 < circularity < 1.5)
                    
                    if(validCnt) and y < lowest_y :
                        myDistFeet = calculateDistanceFeet(w)
                        #print(cntArea, circularity, ratio)
                        cnt_to_process = cnt

            x, y, w, h = cv2.boundingRect(cnt_to_process)
            
            M = cv2.moments(cnt_to_process)
            cy = int(M["m01"] / M["m00"])
            cx = int(M["m10"] / M["m00"])

            #Every 100 cnt detections, print X + Y. Mostly just used to guestimate framerate
            if printCount % 100 == 0:
                print('X center:', cx, 'Y center:',cy)
                printCount = 1
            else:
                printCount += 1

            #The /2 and -.6 are entirely arbitrary values. If re-using this code in the future, you will need to re-sample to find those valvues.
            dist = (myDistFeet/2)-.6
            print(dist)
            
            PacketValue['BallX'] = cy
            PacketValue['BallY'] = cx
            PacketValue['Dist'] = dist
            
            last_contour_x = cx
            last_contour_y = cy
            
            #Send packet, then clear values
            PacketQueue.put_nowait(PacketValue)
            cx = ''
            cy = ''
            last_cnt_area = 0
            cnt_to_process = ''
            lowest_y = 1000

            #print(PacketValue)
        except Exception as e:
            print('Error, likely that a ball wasnt found, error: ')

