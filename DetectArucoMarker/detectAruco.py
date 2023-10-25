

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

mtx1 = np.array([[1093, 0, 322], [0, 1109, 204], [0, 0, 1]])
mtx2 = np.array([[1231, 0 ,328], [0, 1257, 188], [0, 0, 1]])
mtxFudge = np.array([[605,0,314],[0,602,200],[0,0,1]])
mtx = mtxFudge
prevAngle = 0

# Detect aruco marker in undistorted image
while True:
    ret,frame = camera.read() # Take an image
    
    grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) # Make the image greyscale for ArUco detection
    corners, ids, rejected = aruco.detectMarkers(grey, aruco_dict) # find the marker

    if ids is not None:
        
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
        
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[0], 0.05, mtx, None) # get translation and rotation vectors
        functionAway = tvec[0][0][2] # gives distance from marker
        functionLeftRight = tvec[0][0][0] # gives distance from center of image
        
        angleFunction = math.atan((functionLeftRight/functionAway))
        
        if (math.degrees(angleFunction) < 0):
            angleFunction /= 0.98 # fudge factor
        else:
            angleFunction /= 1.2 # fudge factor
        lcd.message = f"Marker Detected \nAngle: {math.degrees(angleFunction):.1f}"
    else:
        lcd.message = "No Marker Detected\n                "
    
                    
cv2.destroyAllWindows()
camera.release()


