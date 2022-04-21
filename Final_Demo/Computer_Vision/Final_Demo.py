#Final Demo: Allows our robot to follow the maze using computer vision

import cv2 as cv

import numpy as np

import argparse

import statistics

import time

import smbus

import time

import board

import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

import math

from time import sleep

from numpy import asarray

from PIL import Image

import picamera 

#bus =smbus.SMBus(1)

#address =0x04

kernel = np.ones((5,5),np.uint8)

quadrantNum = 0

tapeDetected = False                            #Boolean output for status of tape

endOfLine = False                               #Boolean output for end of tape

blindSpot = 12                                  #Length of blindspot in inches (Need to change based on experiment)
                                         
 

#output function to arduino

def writeNumber(value):

    bus.write_byte(address,value)

    return -1

#Function that runs the camera

def takeAPic():

    with picamera.PiCamera() as camera:
        camera.resolution = (320,240) #320,240
        camera.framerate = 24
        time.sleep(2)
        output = np.empty((240,320,3),dtype=np.uint8) #
        camera.capture(output,'bgr') #needs to be in bgr
        im = np.array(output)
        #im.save('picTake.jpg')
        #im.save('/home/pi/Computer_Vision_Files/FinalDemoPic.jpg')
        #cv.imshow("Field of vision",im)
        #cv.waitKey(5) 
        #cv.destroyAllWindows()
        return im

#converts image to HSV

def HSVFunction(picTaken):
    image2HSV = cv.cvtColor(picTaken,cv.COLOR_BGR2HSV)                             #converts from BGR to HSV
    lower_blue = np.array([104,50,50])    #104                                    #Lower value of blue hue (Need to fix)
    upper_blue = np.array([128,255,255])  #128                                   #Upper value of blue hue (Need to fix)
    mask = cv.inRange(image2HSV,lower_blue, upper_blue)                          #Mask for yellow object
    res = cv.bitwise_and(image2HSV ,image2HSV , mask=mask)                       #bitwise and masks original image
    results = cv.imwrite('/home/pi/Computer_Vision_Files/Masked_Final.jpg',res)

    #Rectangular mask for top of camera
    #topMask = np.zeros(image2HSV.shape[:2], np.int8)
    #cv.rectangle(topMask,(0,384),(1024,768),255,-1)                              #mask that covers top portion of screen
    #results = cv.imwrite('/home/pi/Computer_Vision_Files/topMask.jpg',topMask)
    #cv.imshow("Rectangle Mask",topMask)
    #cv.waitKey(5) 
    #cv.destroyAllWindows()
    #finalImage = cv.bitwise_and(res,res,mask = topMask)
    

    cv.imshow('finalImage',res) #finalImage
    #cv.imshow('Field of view',picTaken)
    #cv.imshow('mask',mask)                                                       #shows mask (diagnostic purpuses only)
    #cv.imshow('Masked Image',res)                                                #Shows masked image
    cv.waitKey(5)                                                                 #shows field of view
    cv.destroyAllWindows()
    #results = cv.imwrite('/home/pi/Computer_Vision_Files/maskedImage.jpg',finalImage)
    #print('shape of array',np.shape(image2Process))

    return res

#function to turn the image to grayscale

def grayScaleFun(maskedImage):
    gray = cv.cvtColor(maskedImage,cv.COLOR_BGR2GRAY)
    return gray

#Function to threshold image

def threshold(grayImage):
    _,threshImage = cv.threshold(grayImage,75,255,cv.THRESH_BINARY)
    threshImage2 = cv.morphologyEx(threshImage, cv.MORPH_CLOSE, kernel)
    #thresh = cv.imwrite('/home/pi/Computer_Vision_Files/threshImage.jpg',threshImage2)
    arraySize = np.shape(threshImage2)
    #print('shape of array',arraySize)
    cv.imshow("THRESHOLD", threshImage2)
    cv.waitKey(500)
    cv.destroyAllWindows()
    return threshImage2

#Function that finds center of tape on screen

def tapeFinder(threshImage2):
    imageLocation = np.argwhere(threshImage2)
    imageArray = np.array(imageLocation)
    ImageLocationY = np.mean(imageArray, axis =0)

    #Checks Tape Location and returns colun of its location
    if (np.count_nonzero(threshImage2) == 0):
        print('No marker found')
        colNum = 7                                             #This output can be used to tell robot to spin                                  
    else:
        if(ImageLocationY[1]<107):                             #This tells the robot to turn left
            colNum = 0
        if(ImageLocationY[1]>=107 and ImageLocationY[1]<=213): #This tells the robot to drive straight
            colNum = 1
        if(ImageLocationY[1] >213):                            #This tells the robot to turn right
            colNum = 2
    print('Column',colNum)
    return colNum

#Main
picTaken=takeAPic()
maskedImage = HSVFunction(picTaken)
grayImage = grayScaleFun(maskedImage)
threshImage = threshold(grayImage)
tapeFinder(threshImage)

        
