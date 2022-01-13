import json
import math
import queue
import socket
import sys
import threading
import time
from threading import Thread

import cv2
import numpy as np
from cscore import CameraServer
from networktables import NetworkTables

#Below link has a barebones version, useful for getting the camera server stuff
#https://docs.wpilib.org/en/stable/docs/software/vision-processing/wpilibpi/basic-vision-example.html

if __name__ == "__main__":
    print('2022 Ball Vision Yellow Starting')
    team = 138
    cameraConfig = ''
    
    with open('/boot/frc.json') as f:
        cameraConfig = json.load(f)
        camera = cameraConfig['cameras'][0]

    #print(cameraConfig)
    width = camera['width']
    height = camera['height']
    
    cs = CameraServer.getInstance()
    camera = cs.startAutomaticCapture()
    
    #Important to configure pixel format, otherwise the camera server will expect jpeg capture while the PSEye uses yuyv
    cameraConfig['pixel format'] = 'yuyv'
    camera.setConfigJson(json.dumps(cameraConfig))
    
    input_stream = cs.getVideo()
    output_stream = cs.putVideo('Processed', width, height)

    # Table for vision output information
    vision_nt = NetworkTables.getTable('Vision')

    # Allocating new images is very expensive, always try to preallocate
    img = np.zeros(shape=(640, 480, 3), dtype=np.uint8)

    # Wait for NetworkTables to start
    time.sleep(0.5)

    print('Setup steps complete')

    while True:
        print('Starting image detection')
        start_time = time.time()
        frame_time, input_img = input_stream.grabFrame(img)
        output_img = np.copy(input_img)

        # Notify output of error and skip iteration
        if frame_time == 0:
            output_stream.notifyError(input_stream.getError())
            continue

        img = cv2.imread(output_img) 
        #plt.imshow(img)

        #hue = [0, 22]
        #sat = [110, 255]
        #val = [53, 255]  

        #Yellow Ball params
        yelHue = [24,49]
        yelSat = [92,255]
        yelVal = [110,255]

        # we only care about low portion of frame
        cutOffHeight = height * .5

        out = img
        #out = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #cv2.inRange(out, (hue[0], sat[0], val[0]),  (hue[1], sat[1], val[1]))

        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(img, (yelHue[0], yelSat[0], yelVal[0]),
                            (yelHue[1], yelSat[1], yelVal[1]))
        contours, hiearchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)

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
            approximateShape = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)

            x, y, w, h = cv2.boundingRect(cnt)
            ratio = float(w) / h

            perimeter = cv2.arcLength(cnt, True)


            # finding center point of shape
            M = cv2.moments(cnt)
            if M['m00'] != 0.0:
                x = int(M['m10']/M['m00'])
                y = int(M['m01']/M['m00'])

        


            validCnt = True 
            #validCnt &= (y > cutOffHeight)
            #validCnt &= (cntArea > 20) 
            validCnt &= (len(approximateShape) >= 8)
            validCnt &= (ratio > .2) and (ratio < 3.5)
            validCnt &= cntArea > 10000
            
            circularity = 0
            if(perimeter == 0):
                validCnt = False 
            else:
                circularity = 4*math.pi*(cntArea/(perimeter*perimeter))
                validCnt &= (.5 < circularity < 1.5)
            

            if(validCnt):
                print(cntArea, circularity, ratio)
                con.append(cnt)



        contimage = img

        # mask out rectange
        #cv2.rectangle(contimage, (0, 0), (width, int(cutOffHeight)), (0, 0, 0), -1)

        conCount = 0
        for cnt in con:
            conCount = conCount + 1
            x, y, w, h = cv2.boundingRect(cnt)
            
            M = cv2.moments(cnt)
            cy = int(M["m01"] / M["m00"])
            cx = int(M["m10"] / M["m00"])
            centerX = cx 
            centerY = cy
            centerX = int(centerX)
            centerY = int(centerY)

        print('Passed image processing')
        print(centerX, centerY)




            # draw contour
            #cv2.drawContours(contimage, cnt, -1, (0, 255, 0), 20)

            #cv2.putText(contimage, 'X', (centerX, centerY), cv2.FONT_HERSHEY_SIMPLEX, 4, (0, 0, 255), 3, cv2.LINE_AA)


        # put header text 
        #cv2.putText(contimage,'Ball Vision Camera - Image ' + str(index),(0,100), cv2.FONT_HERSHEY_SIMPLEX, 4, (0, 255, 0), 14, cv2.LINE_AA)
        #cv2.putText(contimage,str(len(con)) + " Ball(s) Detected",(0,260), cv2.FONT_HERSHEY_SIMPLEX, 4, (0, 255, 0), 14, cv2.LINE_AA)





        #print("writing image " + str(index))

        #cv2.imwrite("output" + str(index) + ".jpg", contimage)


