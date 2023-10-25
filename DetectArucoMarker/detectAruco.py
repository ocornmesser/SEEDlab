# do it
#this code looks at an aruco with computer vision and decides which quadrant of the screen it is in. Based on this it outputs a
#string of ascii values to the arduino so that it can read in these values and complete its section of the localization and control

#hardware connections are an LCD and SCL and SDL so that the i2c works with this and the arduino

# importing libraries
from time import sleep
import numpy as np
import cv2
from cv2 import aruco
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import queue
import threading
from random import random
from smbus2 import SMBus
import math
#I2C address
ARD_ADDR = 8
i2c = SMBus(1)
# upload aruco IDs
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

# globals
pixelWidth = 640
pixelHgt = 500


lcd_columns = 16
lcd_rows = 2
i2c = busio.I2C(board.SCL, board.SDA)
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.color = [10,10,10]
camera = cv2.VideoCapture(0) # Initialize the camera
sleep(0.5) # wait for image to stabilize
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*8,3), np.float32)
objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)
'''
#variables to determine number of captures
captures = 0
max_captures = 20 #adjust as needed
mtxHolder = []
while True:
    ret,frame = camera.read() # Take an image
    if ret:
        
        cv2.imshow('nomarker', frame)
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
                break
                
        grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) # Make the image greyscale for ArUco detection
        ret, corners = cv2.findChessboardCorners(grey, (8,6), None) # find the marker
        if ret:
            corners2 = cv2.cornerSubPix(grey,corners,(11,11),(-1,-1),criteria) # more exact than integers
            imgpoints.append(corners2)
            objpoints.append(objp) # 3D real world points
            captures += 1
            cv2.drawChessboardCorners(grey, (8,6), corners2, ret)
            cv2.imshow("image",grey) # show image
            cv2.waitKey(500)

             k = cv2.waitKey(1) & 0xFF
            
            if k == ord('q'):
                break
            
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, grey.shape[::-1],None,None) # get camera matrix
            #print(mtx)
            mtxHolder.append(mtx)
            print(f"rvecs: {rvecs}")
            print(f"tvecs: {tvecs}")
            if captures >= max_captures:
                break


avgMtx = np.zeros(mtxHolder[0].shape)

skipCount = 0
numMtx = 0
for mtx in mtxHolder:
    print('in loop')
    avgMtx += mtx
    numMtx += 1
    
print(f"avgMtx = {avgMtx}")
print(f"numMtx = {numMtx}")
avgMtx /= numMtx
print()
print("average mtx:")
print(avgMtx)
'''
mtx1 = np.array([[1093, 0, 322], [0, 1109, 204], [0, 0, 1]])
mtx2 = np.array([[1231, 0 ,328], [0, 1257, 188], [0, 0, 1]])
mtxFudge = np.array([[605,0,314],[0,602,200],[0,0,1]])
mtx = mtxFudge
prevAngle = 0
# Detect aruco marker in undistorted image
while True:
    ret,frame = camera.read() # Take an image
    #if ret:
        #cv2.imshow('image', frame)
    #k = cv2.waitKey(1) & 0xFF
    #if k == ord('q'):
        #break
    grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) # Make the image greyscale for ArUco detection
    corners, ids, rejected = aruco.detectMarkers(grey, aruco_dict) # find the marker

    if ids is not None:
        

        #cv2.aruco.drawDetectedMarkers(grey, corners)
        #cv2.aruco.drawAxis(grey, mtx, None, rvecs_undist, tvecs_undist, 0.1)
        cv2.cornerSubPix(grey,corners[0][0],(11,11),(-1,-1),criteria)

        (topRight, bottomRight, bottomLeft, topLeft) = corners[0][0] # get coordinates of verticies
        xm = [0,0]
        ym = [0,0]
        xm[0] = ((topRight[0] - topLeft[0])/2.0) + topLeft[0] # find top midpoint
        xm[1] = ((bottomRight[0] - bottomLeft[0])/2.0) + bottomLeft[0] #find bottom midpoint
        xMid = ((xm[1]-xm[0])/2) + xm[0] # find x midpoint
        ym[0] = ((topLeft[1] - bottomLeft[1])/2.0) + bottomLeft[1] # find left midpoint
        ym[1] = ((topRight[1] - bottomRight[1])/2.0) + bottomRight[1] # findright midpoint
        yMid = ((ym[1]-ym[0])/2) + ym[0] # find y midpoint
        pixelLength = topRight[0] - topLeft[0]
        #print(f"pixelLength = {pixelLength}")
        distanceOut = 2910 / ( pixelLength)
        distanceSide = (xMid - 320) / 8.7
        #print(f"distance away math= {distanceOut}")
        #print(f"distance side = {distanceSide}")
        #distanceSide = (xMid - 320)
        angle = math.atan(distanceSide/distanceOut)
        #print(f"angle with math: {math.degrees(angle)*1.13}")
        #q.put(math.degrees(angle))
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[0], 0.05, mtx, None)
        functionAway = tvec[0][0][2]
        functionLeftRight = tvec[0][0][0]
        angleFunction = math.atan((functionLeftRight/functionAway))
        if (math.degrees(angleFunction) < 0):
            angleFunction /= 0.98
        else:
            angleFunction /= 1.2
        lcd.message = f"Marker Detected \nAngle: {math.degrees(angleFunction):.1f}"
    else:
        lcd.message = "No Marker Detected\n                 "

    
                    
cv2.destroyAllWindows()
camera.release()


