import math
import time
import glob
import re
from sqlite3 import Time

import cv2
import numpy as np

vision_directory = 'src/main/java/frc/robot/vision/'

def processSimple(input_img):

    #Red Ball
    '''
    redHue = [0, 12]
    redSat = [109, 255]
    redVal = [58, 255]  
    '''

    #Blue ball
    blueHue = [85, 109]
    blueSat = [161, 255]
    blueVal = [37, 255]  

    #Yellow Ball params
    #yelHue = [18,49]
    #yelSat = [52,255]
    #yelVal = [166,255]

    #Creating settings for blur filter
    radius = 18
    ksize = (2 * round(radius) + 1)

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

    #Used to pick the lowest detected contour in the image which is most likely the closest ball
    lowest_y = 1000

    last_contour_x = 1000
    last_contour_y = 1000
    current_frame = 0
    frame_memory = 0
    contour_found_once = False

    params = cv2.SimpleBlobDetector_Params()
    
    # Set Area filtering parameters
    
    params.filterByArea = True
    params.minArea = 100
    
    # Set Circularity filtering parameters
    params.filterByCircularity = True
    params.minCircularity = 0.8
    
    # Set Convexity filtering parameters
    params.filterByConvexity = False
    params.minConvexity = 0
        
    # Set inertia filtering parameters
    params.filterByInertia = False
    params.minInertiaRatio = 0.01
    
    curTime = time.time()
    oldTime = ''
    
    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)
    blank = np.zeros((1, 1))

    print('Blue ball vision setup complete')

    #Create info for packet
    try:
        current_frame += 1
        if current_frame % 60 == 0:
            oldTime = curTime
            curTime = time.time()
            print(curTime - oldTime)
        
        #start_time = time.time() #Use this to get FPS below

        input_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)
        input_img = cv2.blur(input_img, (ksize, ksize))

        mask = cv2.inRange(input_img, (blueHue[0], blueSat[0], blueVal[0]),
                            (blueHue[1], blueSat[1], blueVal[1]))

        inverted_img = cv2.bitwise_not(mask)    
        # Detect blobs
        '''
        keypoints = detector.detect(inverted_img)
        
        blank = np.zeros((1, 1))
        blobs = cv2.drawKeypoints(mask, keypoints,np.array([]),(0, 0, 255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        #TODO: Find way to fill in  the drawn keypoints
        cv2.imwrite('blobs.jpg', blobs)
        print('Blobs')
        time.sleep(1)
        '''

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)

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
                    #print(cntArea, circularity, ratio)
                    cnt_to_process = cnt

        x, y, w, h = cv2.boundingRect(cnt_to_process)
        
        M = cv2.moments(cnt_to_process)
        cy = int(M['m01'] / M['m00'])
        cx = int(M['m10'] / M['m00'])

        print('X center:', cx, 'Y center:',cy)

        last_contour_x = cx
        last_contour_y = cy

        return cnt_to_process, 'X center: ' + str(cx) + ' Y center: ' + str(cy)

    except Exception as e:
        print('Exception ', e)

def processBlob(input_img):

    #Red Ball
    '''
    redHue = [0, 12]
    redSat = [109, 255]
    redVal = [58, 255]  
    '''

    #Blue ball
    blueHue = [84.9, 111]
    blueSat = [112, 255]
    blueVal = [0, 255]  

    #Yellow Ball params
    #yelHue = [18,49]
    #yelSat = [52,255]
    #yelVal = [166,255]

    #Creating settings for blur filter
    radius = 5.5
    ksize = (2 * round(radius) + 1)

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

    #Used to pick the lowest detected contour in the image which is most likely the closest ball
    lowest_y = 1000

    last_contour_x = 1000
    last_contour_y = 1000
    current_frame = 0
    frame_memory = 0
    contour_found_once = False
    
    dark_blobs = False
    min_area = 10
    circularity = [.70, 1]

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
    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)
    blank = np.zeros((1, 1))

    draw_contours = ''
    first_run = True
    res = ''

    print('Blue ball blob vision setup complete')

    #Create info for packet
    try:
        
        #start_time = time.time() #Use this to get FPS below

        # Notify output of error and skip iteration

        input_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)
        input_img = cv2.blur(input_img, (ksize, ksize))

        mask = cv2.inRange(input_img, (blueHue[0], blueSat[0], blueVal[0]),
                            (blueHue[1], blueSat[1], blueVal[1]))

        #inverted_img = cv2.bitwise_not(mask)
        
        keypoints = detector.detect(mask)
        if keypoints:
            print(keypoints)
            x = keypoints[0].pt[0]
            y = keypoints[0].pt[1]
            print('X: %s, Y: %s' % (x, y))
            blank = np.zeros((1, 1))
            blobs = cv2.drawKeypoints(mask, keypoints,np.array([]),(0, 0, 255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            cv2.imwrite('Blobs.jpeg', blobs)
        

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)

        # Sort contours by area size (biggest to smallest)
        cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

        return blobs, 'X center: ' + str(x) + ' Y center: ' + str(y)

    except Exception as e:
        print('Exception', e)

def processHough(input_img):
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

    cirlces = []

    print('Blue ball vision setup complete')

    try:
        
        #start_time = time.time() #Use this to get FPS below

        # Notify output of error and skip iteration
        
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

            print('x: ', i[0], 'y: ', i[1])
        
        
        cv2.imwrite('Hough.jpg', input_img)
        print('Writing hough circles...')
        
        '''
        input_img = cv2.blur(input_img, (ksize, ksize))

        mask = cv2.inRange(input_img, (blueHue[0], blueSat[0], blueVal[0]),
                            (blueHue[1], blueSat[1], blueVal[1]))
        '''

        return input_img
            
    except Exception as e:
        print('Error: ', e)

print('Local testing starting')

sample_directory = 'Vision Samples/'
output_directory = 'testing_output/'
file = '*'
for name in glob.glob(vision_directory + sample_directory + file):
    file = cv2.imread(name)

    #Process via simple mode

    filename = vision_directory + output_directory + 'simple/' + name[name.find('\\'):name.find('.')] + '-processed' + '.jpg'
    contour, text = processSimple(file)
    image_simple = cv2.drawContours(file, [contour], -1, (255, 255, 255), 10)
    cv2.putText(image_simple, text, (10, 25), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)
    
    cv2.imwrite(filename, image_simple)

    # Write with time stamp instead
    # time.strftime('%Y-%m-%dT%H-%M-%S', time.localtime()) + '-' + str(round(time.time() % 1 * 1000)) + '.jpg'

    filename = vision_directory + output_directory + 'blob/' + name[name.find('\\'):name.find('.')] + '-processed' + '.jpg'
    image_blob, text = processBlob(file)
    cv2.imwrite(filename, image_blob)

    filename = vision_directory + output_directory + 'hough/' + name[name.find('\\'):name.find('.')] + '-processed' + '.jpg'
    image_hough = processHough(file)
    cv2.imwrite(filename, image_hough)
