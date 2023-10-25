
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
camera = cv2.VideoCapture(0) # Initialize the camera
sleep(0.5) # wait for image to stabilize
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*8,3), np.float32)
objp[:,:2] = np.mgrid[0:6,0:8].T.reshape(-1,2)
captures = 0
max_captures = 20 #adjust as needed
mtxHolder = []
while True:
    ret,frame = camera.read() # Take an image
    if ret:
        '''
        cv2.imshow('nomarker', frame)
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
                break
        '''
                
        grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) # Make the image greyscale for ArUco detection
        ret, corners = cv2.findChessboardCorners(grey, (6,8), None) # find the marker
        if ret:
            corners2 = cv2.cornerSubPix(grey,corners,(11,11),(-1,-1),criteria) # more exact than integers
            imgpoints.append(corners2)
            objpoints.append(objp) # 3D real world points
            captures += 1
            cv2.drawChessboardCorners(grey, (6,8), corners2, ret)
            cv2.imshow("image",grey) # show image
            cv2.waitKey(500)

            k = cv2.waitKey(1) & 0xFF
            
            if k == ord('q'):
                break
            
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, grey.shape[::-1],None,None) # get camera matrix
            print(mtx)
            mtxHolder.append(mtx)
            
            if captures >= max_captures:
                break
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((6*8,3), np.float32)
        objp[:,:2] = np.mgrid[0:6,0:8].T.reshape(-1,2)
            


avgMtx = np.zeros(mtxHolder[0].shape)

skipCount = 0
numMtx = 0
for mtx in mtxHolder:
    #print('in loop')
    avgMtx += mtx
    numMtx += 1
    
#print(f"avgMtx = {avgMtx}")
#print(f"numMtx = {numMtx}")
avgMtx /= numMtx
print()
print("average mtx:")
print(avgMtx)
