#Mini Poject: Computer Vision
#this program will monitor field of view in front of camera for user marker and send output of user request to other subsystems
import cv2 as cv
import numpy as np
import argparse
import statistics
import time
import smbus
from time import sleep
from numpy import asarray
from PIL import Image
from picamera import PiCamera
bus =smbus.SMBus(1)
address =0x04
kernel = np.ones((5,5),np.uint8)
quadrantNum = 0

#output function to arduino
def writeNumber(value):
    bus.write_byte(address,value)
    return -1
    
#Function that runs the camera
def takeAPic():
    camera = PiCamera()
    camera.start_preview()
    sleep(5)
    camera.capture('/home/pi/mkdir/miniImage.jpg') #Stores image in mkdir
    camera.stop_preview()
    camera.close()

#function to calibrate the camera
def calibration():
    camera = PiCamera(resolution = (1289,720), framerate = 30)
    #Set ISO to desired value
    camera.iso = 100
    #wait for auto gain to settle
    sleep(2)
    #Seeting values
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    g = camera.awb_gains
    camera.awb_mode = 'off'
    camera.awb_gains = g
    camera.close()

#converts image to HSV
def HSVFunction():
    image2Process = cv.imread('/home/pi/mkdir/miniImage.jpg',cv.IMREAD_UNCHANGED) 
    image2HSV = cv.cvtColor(image2Process,cv.COLOR_BGR2HSV)                      #converts from BGR to HSV
    lower_yellow = np.array([17,50,50])                                          #Lower value of yellow hue
    upper_yellow = np.array([38,255,255])  #changed from 30                      #Upper value of yellow hue
    mask = cv.inRange(image2HSV,lower_yellow, upper_yellow)                      #Mask for yellow object
    res = cv.bitwise_and(image2HSV ,image2HSV , mask=mask)                       #bitwise and masks original image
    #cv.imshow('mask',mask)                                                      #shows mask (diagnostic purpuses only)
    #cv.imshow('Masked Image',res)                                                #Shows masked image
    #cv.waitKey(1)                                                            #shows results
    results = cv.imwrite('/home/pi/mkdir/maskedImage.jpg',res)
    #cv.destroyAllWindows()

#function to turn the image to grayscale
def grayScaleFun():
    maskedImage = cv.imread('/home/pi/mkdir/maskedImage.jpg')
    gray = cv.cvtColor(maskedImage,cv.COLOR_BGR2GRAY) 
    grayImage = cv.imwrite('/home/pi/mkdir/grayImage.jpg',gray)

#Function to threshold image
def threshold():
    image2Thresh= cv.imread('/home/pi/mkdir/grayImage.jpg')
    _,threshImage = cv.threshold(image2Thresh,75,255,cv.THRESH_BINARY)
    thresh = cv.imwrite('/home/pi/mkdir/threshImage.jpg',threshImage)
    #cv.imshow("THRESHOLD", threshImage)
    #cv.waitKey(5)
    #cv.destroyAllWindows()

#Function to detect quadrant of marker
def userQuadrant():
    markerLocation= cv.imread('/home/pi/mkdir/threshImage.jpg')
    locationArray =asarray(markerLocation)                                  #Stores image as numpy array
    imageLocation = np.argwhere(locationArray)                              #Gives location where pixels are nonzero (Replaced nonzero to get rid of tuples)
    imageArray = np.array(imageLocation)                                    #Stores location of image in numpy array
    #print(np.size(imageLocation))
    #print(np.shape(imageArray))
    #ImageLocationX = np.mean(imageArray, axis =1)                           #average of x coordinates (Needs Work)
    #print("image locationx", ImageLocationX)
    ImageLocationY = np.mean(imageArray, axis =0)                           #average of y coordinates (Needs Work)                         
    print("image locationy", ImageLocationY)

    if (np.count_nonzero(locationArray) == 0):                              #Checks to see if marker is there
        print('No markers found')
    else:


        if (ImageLocationY[1] > 256 and ImageLocationY[0]<= 192):                     #Quadrant 1
            quadrantNum = 1

        if (ImageLocationY[1] <=256 and ImageLocationY[0] <= 192):                     #Quadrant 2
            quadrantNum = 2

        if (ImageLocationY[1]<=256 and ImageLocationY[0] > 192):                      #Quadrant 3
            quadrantNum = 3

        if (ImageLocationY[1]> 256 and ImageLocationY[0] > 192):                      #Quadrant 4
            quadrantNum = 4

        print(quadrantNum)
        writeNumber(quadrantNum)                                                 #Function to send user output to arduino

#function that implements program
def markerQuad():
    start_time = time.time()
    seconds = 20
    while True:
        current_time = time.time()
        elapsed_time = current_time - start_time
        calibration()
        takeAPic()
        HSVFunction()
        grayScaleFun()
        threshold()
        userQuadrant()
        print('Complete')
        if elapsed_time > seconds:
            break

markerQuad()



