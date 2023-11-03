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

camera = cv2.VideoCapture(0) # Initialize the camera
sleep(0.5) # wait for image to stabilize
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


mtx = np.array([[665.509,   0,     326.64769],
                [   0,  665.4425,   226.725 ],
                [   0,      0,        0     ]])
dist = np.array([0.1241, -0.928, 0.0038768, -0.0001471, 1.3405])
prevAngle = 0

# Detect aruco marker in undistorted image
while True:
    ret,frame = camera.read() # Take an image
    
    grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) # Make the image greyscale for ArUco detection
    corners, ids, rejected = aruco.detectMarkers(grey, aruco_dict) # find the marker

    if ids is not None:
        
        cv2.cornerSubPix(grey,corners[0][0],(11,11),(-1,-1),criteria)
        
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[0], 0.1, mtx, dist) # get translation and rotation vectors - second arg is aruco size
        functionAway = tvec[0][0][2] # gives distance from marker
        functionLeftRight = tvec[0][0][0] # gives distance from center of image
        
        angleFunction = math.atan((functionLeftRight/functionAway))
        print(f"distance from camera: {tvec[0][0][2]}")
        print(f"distance left/right: {tvec[0][0][0]}")
        print(f"distance up/down: {tvec[0][0][1]}")
        print(f"angle: {math.degrees(angleFunction)}")

        markerInfo = "1 " + f"{math.degrees(angleFunction):.2f} " + f"{tvec[0][0][2]:.2f}"
        print(f"markerInfo: {markerInfo}")
        try:
            # Write a byte to the i2c bus
            command = [ord(character) for character in markerInfo]
            i2c.write_i2c_block_data(ARD_ADDR,0,command)
        except IOError:
            print("Could not write data to the Arduino.")

    else:
        markerInfo = "0 0000 0000"
        try:
            # Write a byte to the i2c bus
            command = [ord(character) for character in markerInfo]
            i2c.write_i2c_block_data(ARD_ADDR,0,command)
        except IOError:
            print("Could not write data to the Arduino.")

    
                    
cv2.destroyAllWindows()
camera.release()
