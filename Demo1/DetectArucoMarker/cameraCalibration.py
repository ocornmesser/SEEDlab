
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
import glob
camera = cv2.VideoCapture(0) # Initialize the camera
sleep(0.5) # wait for image to stabilize
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*8,3), np.float32)
objp[:,:2] = np.mgrid[0:6,0:8].T.reshape(-1,2) * 2.41548
captures = 0
max_captures = 30 #adjust as needed

all_images = glob.glob('capture:*.jpg')

for img in all_images:

    frame = cv2.imread(img)
    grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) # Make the image greyscale for ArUco detection
    ret, corners = cv2.findChessboardCorners(grey, (6,8), None) # find the marker
    
    if ret:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(grey, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

        cv2.drawChessboardCorners(frame, (6,8), corners2, ret)
        cv2.imshow('img', frame)
        cv2.waitKey(1000)

cv2.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, grey.shape[::-1], None, None)

print(f"calibration mtx: {mtx}")
print(f"distortion mtx: {dist}")
        
            
