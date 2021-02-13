import cv2
import numpy as np
import pathlib
import pickle
import os
from os import path
from ximea import xiapi
from datetime import datetime


def calibrateCamera(objectPoints, cornerPoints, leftCamera):

    # TODO - Try calibrateCameraRO
    # ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objectPoints, cornerPoints, (640, 480), None, None)
    ret, mtx, dist, rvecs, tvecs, nop = cv2.calibrateCameraRO(objectPoints, cornerPoints, (1280, 1024), 6, None, None)


    filePath = str(pathlib.Path(__file__).parent.absolute())
    filePath += "/calibration/"

    if leftCamera == True:
        filePath += "left.pkl"
    else:
        filePath += "right.pkl"

    with open(filePath, 'wb') as handle:
        pickle.dump(mtx, handle)
        pickle.dump(dist, handle)
        pickle.dump(rvecs, handle)
        pickle.dump(tvecs, handle)

    return ret, mtx, dist, nop
    

def getCorners(image):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY);    

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # ret, corners = cv2.findChessboardCorners(gray, (8, 6), cv2.CALIB_CB_FAST_CHECK)

    ret, cornersSB = cv2.findChessboardCornersSB(gray, (8, 6))

    image = cv2.drawChessboardCorners(image, (8, 6), cornersSB, ret)

    if ret == True:
        return cornersSB
    else:
        print("Warning, corners not found!!!!")
        return None


def deleteImagePair(index):
    filePath = str(pathlib.Path(__file__).parent.absolute())
    leftFilename =  filePath + "/images/left_" + str(index) + ".png"
    rightFilename = filePath + "/images/right_" + str(index) + ".png"

    os.remove(leftFilename)
    os.remove(rightFilename)    

def renameImagePair(index, goodIndex):    
    filePath = str(pathlib.Path(__file__).parent.absolute())
    leftFilename =  filePath + "/images/left_" + str(index) + ".png"
    rightFilename = filePath + "/images/right_" + str(index) + ".png"

    newLeftFilename =  filePath + "/images/left_" + str(goodIndex) + ".png"
    newRightFilename = filePath + "/images/right_" + str(goodIndex) + ".png"

    os.rename(leftFilename, newLeftFilename)
    os.rename(rightFilename, newRightFilename)

def loadData():
    index = 0
    goodIndex = 0

    squareSize = 0.025
    objp = np.zeros((6*8,3), np.float32)        
    objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)
    objp[:,:2] *= squareSize

    objectPoints = []
    leftCornerPoints = []
    rightCornerPoints = []

    filePath = str(pathlib.Path(__file__).parent.absolute())

    leftFilename =  filePath + "/images/left_" + str(index) + ".png"
    rightFilename = filePath + "/images/right_" + str(index) + ".png"

    includeAll = False

    while (path.exists(leftFilename) and path.exists(rightFilename)):
        leftImage = cv2.imread(leftFilename)
        rightImage = cv2.imread(rightFilename)

        leftCorners = getCorners(leftImage)
        rightCorners = getCorners(rightImage)

        cv2.imshow("Left", leftImage)
        cv2.imshow("Right", rightImage)

        if not includeAll:
            ret = cv2.waitKey(0)
        else:
            ret = 0
        
        if ret == 100: #d key pressed
            deleteImagePair(index)        
        else:
            if ret == 97: #a key pressed
                includeAll = True

            if ret != 115: # s key pressed
                leftCornerPoints.append(leftCorners)
                rightCornerPoints.append(rightCorners)
                objectPoints.append(objp)
                renameImagePair(index, goodIndex)

            goodIndex += 1

        index += 1

        leftFilename =  filePath + "/images/left_" + str(index) + ".png"
        rightFilename = filePath + "/images/right_" + str(index) + ".png"

    print(str((index)) + " frames loaded.")
    print(str(len(objectPoints)) + " object points loaded.")
    print(str(len(leftCornerPoints)) + " left corner points loaded.")
    print(str(len(rightCornerPoints)) + " right corner points loaded.")

    return objectPoints, leftCornerPoints, rightCornerPoints
    

def main():


    objectPoints, leftCornerPoints, rightCornerPoints = loadData()

    # calibrate left camera
    ret, lmtx, ldist, lnop = calibrateCamera(objectPoints, leftCornerPoints, True)
    print("Left camera calibration returned: " + str(ret))

    ret, rmtx, rdist, rnop = calibrateCamera(objectPoints, rightCornerPoints, False)
    print("Right camera calibration returned: " + str(ret))

    ret, slmat, sldist, srmat, srdist, R, T, E, F = cv2.stereoCalibrate(objectPoints, leftCornerPoints, rightCornerPoints, lmtx, ldist, rmtx, rdist, (1280, 1024), flags=cv2.CALIB_FIX_INTRINSIC)
    
    print("Stereo calibration returned: " + str(ret))    
    print("R: " + str(R))
    print("T: " + str(T))

    filePath = str(pathlib.Path(__file__).parent.absolute())
    filePath += "/calibration/stereo.pkl"
    
    with open(filePath, 'wb') as handle:
        pickle.dump(slmat, handle)
        pickle.dump(sldist, handle)
        pickle.dump(srmat, handle)
        pickle.dump(srdist, handle)
        pickle.dump(R, handle)
        pickle.dump(T, handle)
        pickle.dump(E, handle)
        pickle.dump(F, handle)


if __name__ == "__main__":
    main()