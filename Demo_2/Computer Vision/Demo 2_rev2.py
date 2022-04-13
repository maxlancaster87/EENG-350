#Demo 1: A program that finds the angle a piece of tape relative to the robot

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

from picamera import PiCamera

from numpy import zeros
 

bus =smbus.SMBus(1)
address =0x20

kernel = np.ones((5,5),np.uint8)

quadrantNum = 0

tapeDetected = 0                            #Boolean output for status of tape

angleoftape = 180                               #Global variable for angle from tape

distanceFromTape = 50                           #Global variable for distance from tape

endOfLine = False                               #Boolean output for end of tape

blindSpot = 12                                  #Length of blindspot in inches (Need to change based on experiment)


                                        
 

#output function to arduino

def writeNumber(value):

    bus.write_byte(address,value)

    return -1

   

#Function that runs the camera

def takeAPic():

    camera = PiCamera()

    #camera.start_preview()

    #sleep(2)

    camera.capture('/home/pi/Computer_Vision_Files/demo1Pic.jpg') #Stores image in mkdir


    
    #camera.stop_preview()

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

    image2Process = cv.imread('/home/pi/Computer_Vision_Files/demo1Pic.jpg',cv.IMREAD_UNCHANGED)

    image2HSV = cv.cvtColor(image2Process,cv.COLOR_BGR2HSV)                      #converts from BGR to HSV

    lower_blue = np.array([98,50,50])    #104                               #Lower value of blue hue (Need to fix)

    upper_blue = np.array([138,255,255])  #120                                  #Upper value of blue hue (Need to fix)

    mask = cv.inRange(image2HSV,lower_blue, upper_blue)                          #Mask for yellow object

    res = cv.bitwise_and(image2HSV ,image2HSV , mask=mask)                       #bitwise and masks original image

    #Rectangular mask for top of camera
    
    topMask = np.zeros(image2HSV.shape[:2], np.int8)

    cv.rectangle(topMask,(0,130),(1024,768),255,-1) #384 230                            #mask that covers top portion of screen

    results = cv.imwrite('/home/pi/Computer_Vision_Files/topMask.jpg',topMask)

    #cv.imshow("Rectangle Mask",topMask)

    #cv.waitKey(2) 

    #cv.destroyAllWindows()

    finalImage = cv.bitwise_and(res,res,mask = topMask)
    
    #cv.imshow('finalImage',finalImage)

    #cv.imshow('Field of view',image2Process)

    #cv.imshow('mask',mask)                                                       #shows mask (diagnostic purpuses only)

    #cv.imshow('Masked Image',res)                                                #Shows masked image

    #cv.waitKey(2)                                                                 #shows field of view

    results = cv.imwrite('/home/pi/Computer_Vision_Files/maskedImage.jpg',finalImage)

    #cv.destroyAllWindows()

    #print('shape of array',np.shape(image2Process))

 

#function to turn the image to grayscale

def grayScaleFun():

    maskedImage = cv.imread('/home/pi/Computer_Vision_Files/maskedImage.jpg')

    gray = cv.cvtColor(maskedImage,cv.COLOR_BGR2GRAY)

    grayImage = cv.imwrite('/home/pi/Computer_Vision_Files/grayImage.jpg',gray)

    #cv.imshow('mask',mask)

#Function to threshold image

def threshold():

    image2Thresh= cv.imread('/home/pi/Computer_Vision_Files/grayImage.jpg')

    _,threshImage = cv.threshold(image2Thresh,75,255,cv.THRESH_BINARY)

    threshImage2 = cv.morphologyEx(threshImage, cv.MORPH_CLOSE, kernel)

    thresh = cv.imwrite('/home/pi/Computer_Vision_Files/threshImage.jpg',threshImage2)


    #cv.imshow("THRESHOLD", threshImage)

    #cv.waitKey(2)

    #cv.destroyAllWindows()

#Function to find the distance from the robot to the tape
def distanceFunction():

    px = 35.86

    leng = 24

    wid = 1
    
    focalLength = (px*leng)/wid                            #  Focal length = 860 (determined experimentally)

    markerImage= cv.imread('/home/pi/Computer_Vision_Files/threshImage.jpg')

    grey = cv.cvtColor(markerImage, cv.COLOR_BGR2GRAY)

    locationArray =asarray(markerImage) 

    imageLocation = np.argwhere(locationArray)

    ImageLocationY = np.mean(imageLocation, axis =1) 

    YCoord = imageLocation[0][0]

    ycenter = 480/2

    ydist = (YCoord)

    



    print("Ydist ",YCoord )#imageLocation -0.8131*ydist+186.83

    pixWidth = cv.countNonZero(grey)/1290   # may need to divide by 36 to get the width? In theory a version of this should work               

    distanceFromTarget = abs(-1.0742*YCoord+216.91)  #distanceFromTarget

    distanceFromTape = abs(distanceFromTarget)

    #(1.4115*(wid*focalLength)/pixWidth)-2.808

    print('The distance from the target is ',distanceFromTarget)    #Output for distance from tape in inches {Need to format} as output to Arduino

    #print('There are', pixWidth)

    writeNumber(int(distanceFromTarget))  #need this vaiable outputted

    #Serial.write(distanceFromTarget)
#Function to detect angle of tape

def tapeAngle():

    fieldOfView = 3   #3 feet

    fovInPixels = 640 #Field of view in pixels

    pixPerInch = 5    #Pixels per inch

    edgeOfImage = 0

   

    markerLocation= cv.imread('/home/pi/Computer_Vision_Files/threshImage.jpg')

    locationArray =asarray(markerLocation)                                  #Stores image as numpy array

    imageLocation = np.argwhere(locationArray)                              #Gives location where pixels are nonzero (Replaced nonzero to get rid of tuples)

    imageArray = np.array(imageLocation)                                    #Stores location of image in numpy array

    ImageLocationXY = np.mean(imageArray, axis =0)                          #array with x and y coordinate of tape                        

    XCoord = ImageLocationXY[1]                                             #changed from [1] to zero

    tapeYCoord = ImageLocationXY[0]

 

 

   

    #print("image locationy", ImageLocationXY)
    writeNumber(int(-99))                           #Start identifier

    if (np.count_nonzero(locationArray) == 0):

            print('No Tape')

            tapeDetected = 0

            writeNumber(int(0))                     #If tape isnt detected

            return tapeDetected

    elif(np.count_nonzero(locationArray) > 10):

            tapeDetected = 1

            print('Tape Detected')

            pi =3.14159

            xcoordinate = XCoord                       #X coordiante from zero

            #print(xcoordinate)

            xcenter = 640/2                            #center of field of x view @ 3 ft

            ycenter = 480/2                            #center of field of y view @ 3 ft

            fldViewPixls = fieldOfView/170

            if (xcoordinate > xcenter):

                centerFromZero = (xcoordinate-xcenter) #X coordiante from center

                LCD_phi = math.atan(centerFromZero/640)*(180/pi)

            else:

                centerFromZero = (xcenter-xcoordinate)#X coordiante from center

                LCD_phi = -math.atan(centerFromZero/640)*(180/pi)

            distanceFunction()

            #LCD_phi = math.atan(centerFromZero/640)*(180/pi)

            #LCD_phi = 0.5*fldViewPixls*(centerFromZero/(centerFromZero-(pixPerInch/2)))*(180/pi)*100

 

            #phi = ((xcenter/centerFromZero)/xcenter)*(180/pi))

            #print('shape of array',np.shape(imageArray))

            #print('Xcoordinate', xcoordinate)

            print('The angle phi is', LCD_phi, ' degrees') 

            #print('center from zero', centerFromZero)

 

        # Modify this if you have a different sized Character LCD
            #lcd_columns = 16
            #lcd_rows = 2

            # Initialise I2C bus.
            #i2c = board.I2C()  # uses board.SCL and board.SDA

            # Initialise the LCD class
            #lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

            #lcd.clear()
            # Set LCD color to red
            #lcd.color = [100, 0, 0]
            #time.sleep(1)
            # Print two line message
            #lcd.message = ("Angle: " + str(LCD_phi))
            # Wait 5s
            #time.sleep(5)


            writeNumber(int(tapeDetected)) #Need these variables outputted
            writeNumber(int(LCD_phi))


def angleChecker():

    calibration()

    takeAPic()

    HSVFunction()

    grayScaleFun()

    threshold()

    tapeAngle()

    #distanceFunction()

    #return tapeDetected
#function to test argwhere output
def testfunction():
    testArray = zeros([5,5])#[640,480]
    #print(testArray)
    testArray[2][2] = 1;
    #print(testArray)
    newVec = (np.argwhere(testArray))
    ycoord = newVec[0][0]
    #print(newVec,np.size(newVec))
    print(ycoord)

while(endOfLine == False):
    angleChecker()
#testfunction()
