import json
import logging
import math
from multiprocessing.sharedctypes import Value
import queue
import socket
import sys
import threading
import time
from sqlite3 import Time

import cv2
import numpy as np

from cscore import CameraServer
from networktables import NetworkTables
from networktables import NetworkTablesInstance

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


def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()

def calculateDistanceFeet(targetPixelWidth):
    # d = Tft*FOVpixel/(2*Tpixel*tanΘ)
    #Target width in feet * 
    Tft = .75
    tanFOV = math.tan(170/2)
    distEst = Tft * 640 / (2 * targetPixelWidth * tanFOV)
    
    # Unsure as to what measurement distEst is producing in the above line, but multiplying it by .32 will return your distance in feet
    distEstFeet = distEst * .32
    #distEstInches = distEstFeet *.32*12
    return (abs(distEstFeet))

if __name__ == "__main__":
    #Avoid touching camera server settings

    print('2022 Ball Vision FMS Starting')
    
    cameraConfig = ''
    #with open('/home/pi/settings.json') as f:
    with open('/boot/frc.json', 'r') as f:
        mycamera = json.load(f)
        cameraConfig = mycamera
        #print(cameraConfig)
        camera = cameraConfig['cameras'][0]
        

    
    server = False
    team = 138
    team_Server = '10.1.38.2'
    cond = threading.Condition()
    notified = [False]
    teamColor = ''

    
    #Network table setup stuff
    NetworkTables.initialize(server=team_Server)
    ntinst = NetworkTablesInstance.getDefault()
    NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

    with cond:
        print("Waiting")
        if not notified[0]:
            cond.wait()


    table = NetworkTables.getTable('SmartDashboard')

    try:
        teamColor = table.getBoolean('selectedColor')

        print(teamColor)
    except Exception as e:
        print('Likely couldnt get color of ball from network table. Exception:', e)

    cameraHue = [85, 122]
    cameraSat = [119, 255]
    cameraVal = [41, 255]  
    
    while teamColor != True or teamColor != False:
        try:
            if teamColor == True:
                #Red
                cameraConfig = {"fps":120,"height":240,"pixel format":"mjpeg","properties":[{"name":"connect_verbose","value":1},{"name":"raw_brightness","value":-8},{"name":"brightness","value":43},{"name":"raw_contrast","value":0},{"name":"contrast","value":0},{"name":"raw_saturation","value":128},{"name":"saturation","value":100},{"name":"raw_hue","value":-40},{"name":"hue","value":0},{"name":"white_balance_temperature_auto","value":True},{"name":"gamma","value":100},{"name":"raw_gain","value":0},{"name":"gain","value":0},{"name":"power_line_frequency","value":1},{"name":"white_balance_temperature","value":4600},{"name":"raw_sharpness","value":1},{"name":"sharpness","value":33},{"name":"backlight_compensation","value":1},{"name":"exposure_auto","value":3},{"name":"raw_exposure_absolute","value":150},{"name":"exposure_absolute","value":3},{"name":"exposure_auto_priority","value":True}],"width":320}

                cameraHue = [128, 168]
                cameraSat = [0, 255]
                cameraVal = [80, 255]  

            else:
                #Blue
                cameraConfig = {"fps":120,"height":240,"pixel format":"mjpeg","properties":[{"name":"connect_verbose","value":1},{"name":"raw_brightness","value":-8},{"name":"brightness","value":43},{"name":"raw_contrast","value":0},{"name":"contrast","value":0},{"name":"raw_saturation","value":128},{"name":"saturation","value":100},{"name":"raw_hue","value":0},{"name":"hue","value":50},{"name":"white_balance_temperature_auto","value":True},{"name":"gamma","value":100},{"name":"raw_gain","value":0},{"name":"gain","value":0},{"name":"power_line_frequency","value":1},{"name":"white_balance_temperature","value":4600},{"name":"raw_sharpness","value":2},{"name":"sharpness","value":33},{"name":"backlight_compensation","value":1},{"name":"exposure_auto","value":3},{"name":"raw_exposure_absolute","value":157},{"name":"exposure_absolute","value":3},{"name":"exposure_auto_priority","value":True}],"width":320}

                cameraHue = [85, 122]
                cameraSat = [119, 255]
                cameraVal = [41, 255]  

        except:
            pass


    cs = CameraServer.getInstance()
    cameraSettings = cs.startAutomaticCapture()

    cameraSettings.setConfigJson(json.dumps(cameraConfig))
    input_stream = cs.getVideo()
    output_stream = cs.putVideo('Processed', 320, 240)
    
    
    SocketThread = SocketWorker(PacketQueue).start()

    #Numpy creates an array of zeros in the size of the image width/height. Its mentioned in documentation this can be performance intensive, and to do it outside the loop
    #Note that the order is height, width in shape
    imgForm = np.zeros(shape=(240, 320, 3), dtype=np.uint8)
    #Creating settings for blur filter
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
    frame_memory = 0
    contour_found_once = False

    #Create info for packet
    PacketValue = {}

    curTime = time.time()
    oldTime = ''
    printCount = 1
    myDistFeet = 0
    
    # Create a detector with the parameters
    blank = np.zeros((1, 1))

    black_mask = cv2.imread('/home/pi/black_mask.png')

    print('Blue ball vision setup complete')

    while True:
        #Create info for packet
        
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

            input_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)
            input_img = cv2.blur(input_img, (ksize, ksize))
            input_img = cv2.flip(input_img, 1)

            mask = cv2.inRange(input_img, (cameraHue[0], cameraSat[0], cameraVal[0]),
                                (cameraHue[1], cameraSat[1], cameraVal[1]))

            _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)

            # Sort contours by area size (biggest to smallest)
            cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

            con = []
            for cnt in cntsSorted:
                # Calculate Contour area
                cntArea = cv2.contourArea(cnt)
                if (cntArea > cnt_area_low):# and (cntArea < cnt_area_high)
                    # Get moments of contour; mainly for centroid
                    M = cv2.moments(cnt)
                    # Get convex hull (bounding polygon on contour)
                    hull = cv2.convexHull(cnt)
                    
                    # calculate area of convex hull
                    hullArea = cv2.contourArea(hull)
                        
                    # Approximate shape
                    approximateShape = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)

                    x, y, w, h = cv2.boundingRect(cnt)
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
                        myDistFeet = calculateDistanceFeet(w)
                        #print(cntArea, circularity, ratio)
                        cnt_to_process = cnt

            x, y, w, h = cv2.boundingRect(cnt_to_process)
            
            M = cv2.moments(cnt_to_process)
            cy = int(M["m01"] / M["m00"])
            cx = int(M["m10"] / M["m00"])

            if printCount % 100 == 0:
                print('X center:', cx, 'Y center:',cy)
                printCount = 1
            
            else:
                printCount += 1

            dist = myDistFeet
            #The /2 and -.6 are entirely arbitrary values. If re-using this code in the future, you will need to re-sample to find those valvues.
            dist = (dist/2)-.6
            print(dist)
            
            PacketValue['BallX'] = cy
            PacketValue['BallY'] = cx
            PacketValue['Dist'] = dist
            
            last_contour_x = cx
            last_contour_y = cy
            
            #print(PacketValue)
            PacketQueue.put_nowait(PacketValue)
            cx = ''
            cy = ''
            last_cnt_area = 0
            cnt_to_process = ''
            lowest_y = 1000

            #print(PacketValue)
        except Exception as e:
            print('Error, likely that a ball wasnt found, error: ')

