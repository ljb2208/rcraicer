import cv2
import numpy as np
import pathlib
from os import path
from ximea_cam import Camera


def main():

    leftCam = Camera("left", "31703351", downsampling='XI_DWN_1x1')
    rightCam = Camera("right", "32703551", downsampling='XI_DWN_1x1')

    leftCam.open()
    rightCam.open()        

    leftImage = leftCam.getImage()    
    rightImage = rightCam.getImage()

    while(True):

        leftGray = cv2.cvtColor(leftImage, cv2.COLOR_BGR2GRAY)
        rightGray = cv2.cvtColor(rightImage, cv2.COLOR_BGR2GRAY)

        leftVal = cv2.Laplacian(leftGray, cv2.CV_64F).var()
        rightVal = cv2.Laplacian(rightGray, cv2.CV_64F).var()

        statusText = "Val: " + str(int(leftVal))        
        leftImage = cv2.putText(leftImage, statusText, (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 1,  (0, 0, 255), 2)

        statusText = "Val: " + str(int(rightVal))
        rightImage = cv2.putText(rightImage, statusText, (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 1,  (0, 0, 255), 2)

        cv2.imshow("Left", leftImage)
        cv2.imshow("Right", rightImage)        

        leftImage = leftCam.getImage()    
        rightImage = rightCam.getImage()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    


    leftCam.close()
    rightCam.close()

if __name__ == "__main__":
    main()