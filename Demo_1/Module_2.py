#Module Two Program
i = 0
import cv2 as cv
import numpy as np
import argparse
import statistics
from time import sleep
from numpy import asarray
from PIL import Image
kernel = np.ones((5,5),np.uint8)
imFound = False
from picamera import PiCamera
from time import sleep
#function that takes a picture with the camera
def takeAPic():
    #from picamera import PiCamera
    #from time import sleep
    camera = PiCamera()
    camera.start_preview()
    sleep(5)
    camera.capture('/home/pi/mkdir/image%s.jpg'%i) #Stores image in mkdir
    camera.stop_preview()
    camera.close()

#part 1
def cv_excercise1():
    takeAPic()
    img = cv.imread('/home/pi/mkdir/image0.jpg')
    edges = cv.Canny(img,100,200)
    cv.imshow('Canny Image',edges)
    imageclosing = cv.imwrite('/home/pi/mkdir/canny.jpg',edges)
    cv.waitKey(5000)
    cv.destroyAllWindows()

#part 2
def cv_excercise2():
    #part a
    takeAPic()
    image2 = cv.imread('/home/pi/mkdir/image0.jpg',cv.IMREAD_UNCHANGED)    #Reads in image
    scale_percent= 50                                                      #Scaler value of 60%
    width = int(image2.shape[1]*scale_percent/100)                         #Scales width of image
    height = int(image2.shape[0]*scale_percent/100)                        #Scales hieght of image
    dim = (width, height)
    resizedImage =cv.resize(image2, dim, interpolation = cv.INTER_AREA)    #Creates resized version of image

    cv.imshow("Scaled down image", resizedImage)                           #displays resized image
    cv.waitKey(5)
    cv.destroyAllWindows()
    #part b
    image2HSV = cv.cvtColor(resizedImage,cv.COLOR_BGR2HSV)                 #converts from BGR to HSV
    lower_yellow = np.array([14,50,50])                                    #Lower value of yellow hue (maybe play with values)
    upper_yellow = np.array([38,255,255])  #changed from 30                                #Upper value of yellow hue
    mask = cv.inRange(image2HSV,lower_yellow, upper_yellow)                #Mask for yellow object
    res = cv.bitwise_and(resizedImage,resizedImage, mask=mask)             #bitwise and masks original image
    cv.imshow('resizedImage',resizedImage)                                 #Shows resized image
    #cv.imshow('mask',mask)                                                #shows mask (diagnostic purpuses only)
    cv.imshow('Masked Image',res)                                          #Shows masked image
    cv.waitKey(5)
    results = cv.imwrite('/home/pi/mkdir/part2result.jpg',res)
    cv.destroyAllWindows()

#part 3
def cv_excercise3():
    image3 = cv.imread('/home/pi/mkdir/part2result.jpg')
    closing = cv.morphologyEx(image3,cv.MORPH_CLOSE, kernel)               #removes dots on inside of image
    cv.imshow("Image 3",image3)
    cv.imshow("Closing",closing)
    cv.waitKey(5)
    cv.destroyAllWindows()
    imageclosing = cv.imwrite('/home/pi/mkdir/part3closing.jpg',closing)   
    blur = cv.blur(closing,(5,5))                                          #blurs image to smooth it out
    opening =cv.morphologyEx(blur,cv.MORPH_OPEN,kernel)                    #removes dots on outside of image
    image2process = cv.imwrite('/home/pi/mkdir/part3closing.jpg',opening)
    #New T
    #erosion = cv.erode(opening,kernel,iterations=1)                        #Erodes Image
    #dilation = cv.dilate(erosion,kernel,iterations=1)                      #Dilates Image
    #gradient = cv.morphologyEx(dilation, cv.MORPH_GRADIENT, kernel)        #Gradient of image
    #tophat = cv.morphologyEx(gradient, cv.MORPH_TOPHAT,kernel)             #tophat of image
    #blackhat = cv.morphologyEx(tophat, cv.MORPH_BLACKHAT,kernel)           #Blackhat of image
    #cv.imshow("Blackhat",blackhat)
    #cv.waitKey(25000)
    #cv.destroyAllWindows()

#part 4
def cv_excercise4():
    cv_excercise2()                                                         #Calls function 2 to resize and mask image
    cv_excercise3()                                                         #Calls functino 3 to process image
    image4 = cv.imread('/home/pi/mkdir/part3closing.jpg')
    grey = cv.cvtColor(image4,cv.COLOR_BGR2GRAY)                            #Converts image to grayscale
    _,threshImage = cv.threshold(grey,75,255,cv.THRESH_BINARY) #127,255     #Does threshold operation on image    
    cv.imshow("THRESHOLD", threshImage)
    cv.waitKey(2)
    cv.destroyAllWindows()
    locationArray =asarray(threshImage)                                     #Stores image as numpy array
    #locationArray = np.array(threshImage)
    imageLocation = np.nonzero(locationArray)                               #Gives location where pixels are nonzero
    imageArray = np.array(imageLocation)                                    #Stores location of image in numpy array
    #print("nonZero array", imageLocation)
    #print("np array", imageArray) 
    ImageLocationX = np.mean(imageArray, axis =1)                           #average of x coordinates
    #print("image locationx", ImageLocationX)
    ImageLocationY = np.mean(imageArray, axis =0)                           #average of y coordinates                          
    #print("image locationy", ImageLocationY)
    Xcoord = np.mean(ImageLocationX)                                        #average of average coordiante
    #print("xcoord", Xcoord)
    Ycoord = np.mean(ImageLocationY)                                        #average of average coordiante
    #print("xcoord", Ycoord)
    #print(image4.shape)
    print("The image is located at", Xcoord, "and", Ycoord, "y")
    cv.imshow("THRESHOLD", threshImage)
    cv.waitKey(3)
    cv.destroyAllWindows()
    return Xcoord#,Ycoord

#part 5
def cv_excercise5():
    imFound = False
    while imFound == False:
        cv_excercise2()                                                          #Calls function 2 to resize and mask image
        cv_excercise3()
        image4 = cv.imread('/home/pi/mkdir/part3closing.jpg')                    #Calls functino 3 to process image
        grey = cv.cvtColor(image4,cv.COLOR_BGR2GRAY)                             #Converts image to grayscale
        _,threshImage = cv.threshold(grey,75,255,cv.THRESH_BINARY) #127,255      #Does threshold operation on image    
        cv.imshow("THRESHOLD", threshImage)
        cv.waitKey(2)
        cv.destroyAllWindows()
        locationArray =asarray(threshImage)                                      #Stores image as numpy array
        #locationArray = np.array(threshImage)
        if (np.count_nonzero(locationArray) == 0):
            print('No markers found')
            print(imFound)
        elif(np.count_nonzero(locationArray) > 10):
            imageLocation = np.nonzero(locationArray)                            #Gives location where pixels are nonzero
            imageArray = np.array(imageLocation)                                 #Stores location of image in numpy array
            #print("nonZero array", imageLocation)
            #print("np array", imageArray) 
            ImageLocationX = np.mean(imageArray, axis =1)                        #average of x coordinates
            #print("image locationx", ImageLocationX)
            ImageLocationY = np.mean(imageArray, axis =0)                        #average of y coordinates                          
            #print("image locationy", ImageLocationY)
            Xcoord = np.mean(ImageLocationX)                                     #average of average coordiante
            #print("xcoord", Xcoord)
            Ycoord = np.mean(ImageLocationY)                                     #average of average coordiante
            #print("xcoord", Ycoord)
            print("The image is located at", Xcoord, "and", Ycoord, "y")
            cv.imshow("THRESHOLD", threshImage)
            cv.waitKey(1)
            cv.destroyAllWindows()
            imFound = True
            print(imFound)

#part 6
def cv_excercise6():
    pi =3.14159
    xcoordinate = cv_excercise4()
    print(xcoordinate)
    xcenter = 512/2
    ycenter = 384/2
    if (xcoordinate > xcenter):
        centerFromZero = (xcoordinate-xcenter)
    else:
        centerFromZero = (xcenter-xcoordinate)
    phi = ((xcenter/centerFromZero)/xcenter)*(180/pi)
    print('The angle phi is', phi, ' degrees')
    
    


takeAPic()
#cv_excercise1()
#cv_excercise2()
#cv_excercise3()
#cv_excercise4()
#cv_excercise5()
#cv_excercise6()                   

