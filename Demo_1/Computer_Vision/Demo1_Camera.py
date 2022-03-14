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
    camera.capture('/home/pi/Computer_Vision_Files/demo1Pic.jpg') #Stores image in mkdir
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
    image2Process = cv.imread('/home/pi/Computer_Vision_Files/demo1Pic.jpg',cv.IMREAD_UNCHANGED) 
    image2HSV = cv.cvtColor(image2Process,cv.COLOR_BGR2HSV)                      #converts from BGR to HSV
    lower_blue = np.array([100,50,50])    #90                                 #Lower value of blue hue (Need to fix)
    upper_blue = np.array([128,255,255])  #128                                   #Upper value of blue hue (Need to fix)
    mask = cv.inRange(image2HSV,lower_blue, upper_blue)                          #Mask for yellow object
    res = cv.bitwise_and(image2HSV ,image2HSV , mask=mask)                       #bitwise and masks original image

    cv.imshow('Field of view',image2Process)
    #cv.imshow('mask',mask)                                                       #shows mask (diagnostic purpuses only)
    #cv.imshow('Masked Image',res)                                                #Shows masked image
    cv.waitKey(5000)                                                             #shows results
    results = cv.imwrite('/home/pi/Computer_Vision_Files/maskedImage.jpg',res)
    cv.destroyAllWindows()
    print('shape of array',np.shape(image2Process))

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
    thresh = cv.imwrite('/home/pi/Computer_Vision_Files/threshImage.jpg',threshImage)
    cv.imshow("THRESHOLD", threshImage)
    cv.waitKey(5)
    cv.destroyAllWindows()
    
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
    XCoord = ImageLocationXY[1]    #changed from [1] to zero
    tapeYCoord = ImageLocationXY[0]


    
    print("image locationy", ImageLocationXY)
    if (np.count_nonzero(locationArray) == 0):
            print('No Tape')
    elif(np.count_nonzero(locationArray) > 10):
            print('Tape Detected')
            pi =3.14159
            xcoordinate = XCoord                       #X coordiante from zero
            #print(xcoordinate)
            xcenter = 640/2                            #center of field of x view @ 3 ft
            ycenter = 480/2                            #center of field of y view @ 3 ft
            fldViewPixls = fieldOfView/170
            if (xcoordinate > xcenter):
                centerFromZero = (xcoordinate-xcenter) #X coordiante from center
                LCD_phi = 2*math.atan(centerFromZero/640)*(180/pi)
            else:
                centerFromZero = (xcenter-xcoordinate)#X coordiante from center
                LCD_phi = -2*math.atan(centerFromZero/640)*(180/pi)
            #LCD_phi = math.atan(centerFromZero/640)*(180/pi)
            #LCD_phi = 0.5*fldViewPixls*(centerFromZero/(centerFromZero-(pixPerInch/2)))*(180/pi)*100

            #phi = ((xcenter/centerFromZero)/xcenter)*(180/pi))
            #print('shape of array',np.shape(imageArray))
            print('Xcoordinate', xcoordinate)
            print('The angle phi is', LCD_phi, ' degrees')
            print('center from zero', centerFromZero)

            # Modify this if you have a different sized Character LCD
            lcd_columns = 16
            lcd_rows = 2

            # Initialise I2C bus.
            i2c = board.I2C()  # uses board.SCL and board.SDA

            # Initialise the LCD class
            lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

            lcd.clear()
            # Set LCD color to red
            lcd.color = [100, 0, 0]
            time.sleep(1)
            # Print two line message
            lcd_string = ("Angle:", + str(LCD_phi))
            # Wait 5s
            time.sleep(5)



def angleChecker():
    calibration()
    takeAPic()
    HSVFunction()
    grayScaleFun()
    threshold()
    tapeAngle()
angleChecker()
