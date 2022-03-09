#Demo 1: A program that finds the angle a piece of tape relative to the robot
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
#bus =smbus.SMBus(1)
#address =0x04
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
    camera.capture('/home/pi/mkdir/demo1Pic.jpg') #Stores image in mkdir
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
    image2Process = cv.imread('/home/pi/mkdir/demo1Pic.jpg',cv.IMREAD_UNCHANGED) 
    image2HSV = cv.cvtColor(image2Process,cv.COLOR_BGR2HSV)                      #converts from BGR to HSV
    lower_blue = np.array([90,50,50])    #219                                   #Lower value of blue hue (Need to fix)
    upper_blue = np.array([128,255,255])  #240                                   #Upper value of blue hue (Need to fix)
    mask = cv.inRange(image2HSV,lower_blue, upper_blue)                          #Mask for yellow object
    res = cv.bitwise_and(image2HSV ,image2HSV , mask=mask)                       #bitwise and masks original image
    cv.imshow('mask',mask)                                                       #shows mask (diagnostic purpuses only)
    cv.imshow('Masked Image',res)                                                #Shows masked image
    cv.waitKey(5000)                                                             #shows results
    results = cv.imwrite('/home/pi/mkdir/maskedImage.jpg',res)
    cv.destroyAllWindows()

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
    
#Function to detect angle of tape
def tapeAngle():
    markerLocation= cv.imread('/home/pi/mkdir/threshImage.jpg')
    locationArray =asarray(markerLocation)                                  #Stores image as numpy array
    imageLocation = np.argwhere(locationArray)                              #Gives location where pixels are nonzero (Replaced nonzero to get rid of tuples)
    imageArray = np.array(imageLocation)                                    #Stores location of image in numpy array
    ImageLocationXY = np.mean(imageArray, axis =0)                          #array with x and y coordinate of tape                         
    tapeXCoord = ImageLocationXY[1]
    tapeYCoord = ImageLocationXY[0]
    print("image locationy", ImageLocationXY)
    if (np.count_nonzero(locationArray) == 0):
            print('No markers found')
    elif(np.count_nonzero(locationArray) > 10):
            pi =3.14159
            xcoordinate = tapeXCoord
            print(xcoordinate)
            xcenter = 512/2
            ycenter = 384/2
            if (xcoordinate > xcenter):
                centerFromZero = (xcoordinate-xcenter)
            else:
                centerFromZero = (xcenter-xcoordinate)
                phi = ((xcenter/centerFromZero)/xcenter)*(180/pi)
                print('The angle phi is', phi, ' degrees')



calibration()
takeAPic()
HSVFunction()
grayScaleFun()
threshold()
tapeAngle()
