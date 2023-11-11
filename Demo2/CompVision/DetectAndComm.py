# importing libraries
from time import sleep
import numpy as np
import cv2
from cv2 import aruco
import board
import busio
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
infoList = []

camera = cv2.VideoCapture(0) # Initialize the camera
sleep(0.5) # wait for image to stabilize
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
parameters = aruco.DetectorParameters()
parameters.minMarkerPerimeterRate = 0.01
#parameters.apdaptiveTreshWinSizeMax = 50


mtx = np.array([[665.509,   0,     326.64769],
               [   0,  665.4425,   226.725 ],
               [   0,      0,        0     ]])
dist = np.array([0.1241, -0.928, 0.0038768, -0.0001471, 1.3405])
mtxFar = np.array([[514.16222047,   0,         328.76355401],
 [  0,         513.44122177, 199.99394471],
 [  0,           0,           1,        ]])
distFar = np.array([[ 0.1199373 , -0.23256955 , 0.00132602 , 0.00246679, 0.0268933 ]])


print("starting")

while True:
    ret,frame = camera.read() # Take an image
    
    grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) # Make the image greyscale for ArUco detection
    corners, ids, rejected = aruco.detectMarkers(grey, aruco_dict, parameters=parameters) # find the marker

    if ids is not None:
        
        cv2.cornerSubPix(grey,corners[0][0],(11,11),(-1,-1),criteria)
        
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[0], 0.1, mtx, dist) # get translation and rotation vectors - second arg is aruco size
        
        
        functionAway = tvec[0][0][2] # gives distance from marker (m). 1 is up/down  and 0 is left/right
        functionLeftRight = tvec[0][0][0] # gives distance from center of image
        
        angleFunction = math.atan(functionLeftRight/(functionAway))
        
        try:
             #Write a byte to the i2c bus
            #command = [ord(character) for character in markerInfo]
            intDegrees = int(math.degrees(angleFunction) * 10)
            intDistAway = int(functionAway * 100)
            infoList = [1, intDegrees, intDistAway]
            print(f"{infoList}")
            i2c.write_i2c_block_data(ARD_ADDR,0,infoList)
        except IOError:
            #print("Could not write data to the Arduino.")
            continue
            

    else:
        print(f"[0 00 00]")
        
        try:
            # Write a byte to the i2c bus
            #command = [ord(character) for character in markerInfo]
            infoList = [0, 0, 0]
            i2c.write_i2c_block_data(ARD_ADDR,0,infoList)
        except IOError:
            print("Could not write data to the Arduino.")
            continue
            

    
                    
cv2.destroyAllWindows()
camera.release()

