#!/usr/bin/python3

import cv2
import numpy as np
import pathlib
from os import path
from ximea_cam import Camera
from datetime import datetime


currentImage = None
leftIndex = 0
rightIndex = 0



def onClick(event,x,y,flags,param):    

    if event == cv2.EVENT_LBUTTONUP:
        if currentImage is not None:
            currentImage.extractDepth(x, y)

def saveImage(image, isLeft):
    global leftIndex
    global rightIndex

    filename = str(pathlib.Path(__file__).parent.absolute()) + "/circle_images/"

    if isLeft == True:
        filename +=  "left_" + str(leftIndex) + ".png"
        leftIndex += 1
    else:
        filename +=  "right_" + str(rightIndex) + ".png"
        rightIndex += 1

        
    cv2.imwrite(filename, image)
    print("Saved image " + filename)

def drawCorners(image):    
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, centres = cv2.findCirclesGrid(gray, (11,4), flags=cv2.CALIB_CB_ASYMMETRIC_GRID)    

    image = cv2.drawChessboardCorners(image, (11,4), centres, ret)

    return ret
    
    

def getStartIndex():
    global leftIndex
    global rightIndex

    filename = str(pathlib.Path(__file__).parent.absolute()) + "/circle_images/"

    leftFilename = filename + "left_" + str(leftIndex) + ".png"
    rightFilename = filename + "right_" + str(rightIndex) + ".png"

    while (path.exists(leftFilename) and path.exists(rightFilename)):
        leftIndex += 1
        rightIndex += 1

        leftFilename = filename + "left_" + str(leftIndex) + ".png"
        rightFilename = filename + "right_" + str(rightIndex) + ".png"

        

def main():
    global currentImage
    global leftIndex
    global rightIndex

    getStartIndex()

    currentImage = None

    leftCam = Camera("left", "31703351", downsampling='XI_DWN_1x1')
    rightCam = Camera("right", "32703551", downsampling='XI_DWN_1x1')    

    leftCam.open()
    rightCam.open()

    cv2.namedWindow("Left")
    cv2.namedWindow("Right")
    cv2.setMouseCallback("Right", onClick)    

    saveImageFlag = False    
    startCapture = False
    lastCapture = datetime.now()

    
    goodImages = 0    

    secondsLeft = 5    

    while (True):            
        
        leftData = leftCam.getImage()
        rightData = rightCam.getImage()                

        leftDataSave = leftData.copy()
        rightDataSave = rightData.copy()

        rightRet = drawCorners(rightData)
        leftRet = drawCorners(leftData)        

        if startCapture == True:
            if rightRet == 0 and leftRet  == 0:
                goodImages += 1

                if goodImages > 10:
                    saveImageFlag = True
            else:
                goodImages = 0
            
            if saveImageFlag == True:
                saveImage(rightDataSave, False)
                saveImage(leftDataSave, True)
                goodImages = 0
                saveImageFlag = False
                startCapture = False
                lastCapture = datetime.now()

        else:
            secondsLeft = 5 - (datetime.now() - lastCapture).total_seconds()

            if secondsLeft <= 0:
                secondsLeft = 0
                startCapture = True            
                

        statusText = "Saved: " + str(leftIndex)
        pointsText = "Corners: " + str(leftRet)
        leftData = cv2.putText(leftData, statusText, (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 1,  (0, 0, 255), 2)
        leftData = cv2.putText(leftData, pointsText, (400, 40), cv2.FONT_HERSHEY_SIMPLEX, 1,  (0, 0, 255), 2)

        if startCapture == False:
            statusText = "Next capture: " + str(int(secondsLeft)) + "s"
        else:
            statusText = "Capturing now....."


        poitnsText = "Corners: " + str(rightRet)
        rightData = cv2.putText(rightData, statusText, (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 1,  (0, 0, 255), 2)
        rightData = cv2.putText(rightData, pointsText, (400, 40), cv2.FONT_HERSHEY_SIMPLEX, 1,  (0, 0, 255), 2)

        cv2.imshow("Left", leftData)
        cv2.imshow("Right", rightData)        

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    leftCam.close()
    rightCam.close()
    
    cv2.destroyAllWindows()



if __name__ == "__main__":
    main()