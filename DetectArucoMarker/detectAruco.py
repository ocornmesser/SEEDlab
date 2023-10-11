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


while True:
    
    ret,frame = camera.read() # Take an image
    if ret:
        grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) # Make the image greyscale for ArUco detection
        imageWithQuad = grey.copy()
        cv2.line(imageWithQuad, (320,500),(320,0), (255,0,0), 2)
        cv2.line(imageWithQuad, (640,250),(0,250), (255,0,0), 2)
        cv2.imshow("image",imageWithQuad) # show image
        corners,ids,rejected = aruco.detectMarkers(grey,aruco_dict) # find the marker
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        if not ids is None:
            #verticies = corners.reshape((4,2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners[0][0] # get coordinates of verticies
            xm = [0,0]
            ym = [0,0]
            xm[0] = ((topRight[0] - topLeft[0])/2.0) + topLeft[0] # find top midpoint
            xm[1] = ((bottomRight[0] - bottomLeft[0])/2.0) + bottomLeft[0] #find bottom midpoint
            xMid = ((xm[1]-xm[0])/2) + xm[0] # find x midpoint
            ym[0] = ((topLeft[1] - bottomLeft[1])/2.0) + bottomLeft[1] # find left midpoint
            ym[1] = ((topRight[1] - bottomRight[1])/2.0) + bottomRight[1] # findright midpoint
            yMid = ((ym[1]-ym[0])/2) + ym[0] # find y midpoint
            #print(f"xMid = {xMid}, yMid = {yMid}")
            if xMid >= (pixelWidth/2): # right half of the camera (east)
                if yMid >= (pixelHgt/2): # bottom right quarter of camera (SE)
                    # send out SE command
                    newDesLocation = '10'
                elif yMid < (pixelHgt/2): # top right quarter of camera (NE)
                    # send out NE command
                    newDesLocation = '00'
            elif xMid <= (pixelWidth/2): # left half of the camera (west)
                if yMid >= (pixelHgt/2): # bottom left quarter of camera (SW)
                    # send out SW command
                    newDesLocation = '11'
                elif yMid < (pixelHgt/2): # top left quarter of camera (NW)
                    # send out NW command
                    newDesLocation = '01'
            # Send it to the thread and arduino
            if newDesLocation != currDesLocation: # Put your own conditional here (i.e. ArUco marker moved)
                q.put(newDesLocation)
                currDesLocation = newDesLocation
                try:
                    # Write a byte to the i2c bus
                    command = [ord(character) for character in newDesLocation]
                    i2c.write_i2c_block_data(ARD_ADDR,0,command)
                except IOError:
                    print("Could not write data to the Arduino.")
camera.release()
