import cv2
import numpy as np
import pathlib
from ximea import xiapi
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

    filename = str(pathlib.Path(__file__).parent.absolute())

    if isLeft == True:
        filename +=  "/images/left_" + str(leftIndex) + ".png"
        leftIndex += 1
    else:
        filename +=  "/images/right_" + str(rightIndex) + ".png"
        rightIndex += 1

        
    cv2.imwrite(filename, image)
    print("Saved image " + filename)

def drawCorners(image):    

    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY);    

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # ret, corners = cv2.findChessboardCorners(gray, (8, 6), cv2.CALIB_CB_FAST_CHECK)

    ret, cornersSB = cv2.findChessboardCornersSB(gray, (8, 6))

    if ret == True:
        # corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # cv2.drawChessboardCorners(image, (8, 6), corners2, ret)                            
        cv2.drawChessboardCorners(image, (8, 6), cornersSB, ret)

    
    return ret
    

def main():
    global currentImage
    global leftIndex
    global rightIndex

    currentImage = None

    leftCam = xiapi.Camera()
    rightCam = xiapi.Camera()


    leftCam.open_device_by_SN('31703351')
    rightCam.open_device_by_SN('32703551')    

    leftCam.set_downsampling('XI_DWN_2x2')
    rightCam.set_downsampling('XI_DWN_2x2')

    leftCam.set_imgdataformat('XI_RGB24')
    rightCam.set_imgdataformat('XI_RGB24')
    leftCam.enable_aeag()
    rightCam.enable_aeag()
    leftCam.enable_auto_wb()
    rightCam.enable_auto_wb()

    
    leftImg = xiapi.Image()
    rightImg = xiapi.Image()

    leftCam.start_acquisition()
    rightCam.start_acquisition()    

    cv2.namedWindow("Left")
    cv2.namedWindow("Right")
    cv2.setMouseCallback("Right", onClick)    

    saveImageFlag = False    
    startCapture = False
    lastCapture = datetime.now()

    goodImages = 0    

    secondsLeft = 5    

    while (True):            
        
        leftCam.get_image(leftImg)
        rightCam.get_image(rightImg)        

        # data_raw = img.get_image_data_raw()
        leftData = leftImg.get_image_data_numpy()
        rightData = rightImg.get_image_data_numpy()

        leftDataSave = leftData.copy()
        rightDataSave = rightData.copy()

        rightRet = drawCorners(rightData)
        leftRet = drawCorners(leftData)        

        if startCapture == True:
            if rightRet == True and leftRet == True:
                goodImages += 1

                if goodImages > 30:
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
        leftData = cv2.putText(leftData, statusText, (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 1,  (0, 0, 255), 2)

        if startCapture == False:
            statusText = "Next capture: " + str(secondsLeft) + "s"
        else:
            statusText = "Capturing now....."

        rightData = cv2.putText(rightData, statusText, (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 1,  (0, 0, 255), 2)

        cv2.imshow("Left", leftData)
        cv2.imshow("Right", rightData)        

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    leftCam.stop_acquisition()
    rightCam.stop_acquisition()

    leftCam.close_device()
    rightCam.close_device()
    cv2.destroyAllWindows()



if __name__ == "__main__":
    main()