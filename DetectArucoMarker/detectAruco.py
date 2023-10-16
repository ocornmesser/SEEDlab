# do it
#this code looks at an aruco with computer vision and decides which quadrant of the screen it is in. Based on this it outputs a
#string of ascii values to the arduino so that it can read in these values and complete its section of the localization and control

#hardware connections are an LCD and SCL and SDL so that the i2c works with this and the arduino

# importing libraries
from time import sleep
import numpy as np
import cv2
from cv2 import aruco
import numpy as np
from time import sleep
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import queue
import threading
from random import random
from smbus2 import SMBus
#I2C address
ARD_ADDR = 8
i2c = SMBus(1)
# upload aruco IDs
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

# globals
pixelWidth = 640
pixelHgt = 500
currDesLocation = '00'

q = queue.Queue()
def myFunction():
    # initialize LCD screen
    lcd_columns = 16
    lcd_rows = 2
    i2c = busio.I2C(board.SCL, board.SDA)
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
    while True:
        if not q.empty():
            location = q.get()
            lcd.color = [10,10,10]
            lcd.message = "Goal Position:\n" + location[0] + ' ' + location[1]

myThread = threading.Thread(target=myFunction,args=())
myThread.start()

camera = cv2.VideoCapture(0) # Initialize the camera
sleep(0.5) # wait for image to stabilize
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)


while True:
    
    ret,frame = camera.read() # Take an image
    if ret:
        grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) # Make the image greyscale for ArUco detection
        cv2.imshow("image",grey) # show image
        corners,ids,rejected = aruco.detectMarkers(grey,aruco_dict) # find the marker
        objpoints.append(objp) # 3D real world points
        cv2.cornerSubPix(grey,corners,(11,11),(-1,-1),criteria) # more exact than integers
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        if not ids is None:
            
            (topLeft, topRight, bottomRight, bottomLeft) = corners[0][0] # get coordinates of verticies

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, grey.shape[::-1],None,None) # get camera matrix
        h, w = grey.shape[:2] # height and width of image
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h)) # new camera matrix without distortion
        dst = cv2.undistort(grey, mtx, dist, None, newcameramtx) # undistort image
        cv2.imshow("image",dst) # show image

camera.release()

'''
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('*.jpg')
 
for fname in images:
img = cv2.imread(fname)
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

# Find the chess board corners
ret, corners = cv2.findChessboardCorners(gray, (7,6),None)

# If found, add object points, image points (after refining them)
if ret == True:
objpoints.append(objp)

cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
imgpoints.append(corners)

# Draw and display the corners
cv2.drawChessboardCorners(img, (7,6), corners2,ret)
cv2.imshow('img',img)
cv2.waitKey(500)

cv2.destroyAllWindows()
'''
