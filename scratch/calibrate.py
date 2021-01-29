import cv2
import numpy as np
import pathlib
from os import path
from ximea import xiapi
from datetime import datetime


def calibrateCamera(objectPoints, cornerPoints, leftCamera):

    # TODO - Try calibrateCameraRO
    # ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objectPoints, cornerPoints, (640, 480), None, None)
    ret, mtx, dist, rvecs, tvecs, nop = cv2.calibrateCameraRO(objectPoints, cornerPoints, (640, 480), 30, None, None)


    filePath = str(pathlib.Path(__file__).parent.absolute())
    filePath += "/calibration/"

    if leftCamera == True:
        filePath += "left"
    else:
        filePath += "right"

    fileName = filePath + "_mtx.txt"
    mtx.tofile(fileName)

    fileName = filePath + "_dist.txt"
    dist.tofile(fileName)

    # fileName = filePath + "_rvecs.txt"
    # rvecs.tofile(fileName)

    # fileName = filePath + "_tvecs.txt"
    # tvecs.tofile(fileName)


    return ret, mtx, dist, nop
    

def getCorners(image):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY);    

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # ret, corners = cv2.findChessboardCorners(gray, (8, 6), cv2.CALIB_CB_FAST_CHECK)

    ret, cornersSB = cv2.findChessboardCornersSB(gray, (8, 6))

    if ret == True:
        return cornersSB
    else:
        print("Warning, corners not found!!!!")
        return None


def loadData():
    index = 0

    objp = np.zeros((6*8,3), np.float32)
    objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

    objectPoints = []
    leftCornerPoints = []
    rightCornerPoints = []

    filePath = str(pathlib.Path(__file__).parent.absolute())

    leftFilename =  filePath + "/images/left_" + str(index) + ".png"
    rightFilename = filePath + "/images/right_" + str(index) + ".png"

    while (path.exists(leftFilename) and path.exists(rightFilename)):
        leftImage = cv2.imread(leftFilename)
        rightImage = cv2.imread(rightFilename)

        leftCornerPoints.append(getCorners(leftImage))
        rightCornerPoints.append(getCorners(rightImage))
        objectPoints.append(objp)

        index += 1

        leftFilename =  filePath + "/images/left_" + str(index) + ".png"
        rightFilename = filePath + "/images/right_" + str(index) + ".png"

    print(str((index - 1)) + " frames loaded.")
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

    ret, slmat, sldist, srmat, srdist, R, T, E, F = cv2.stereoCalibrate(objectPoints, leftCornerPoints, rightCornerPoints, lmtx, ldist, rmtx, rdist, (640, 480))

    print("Stereo calibration returned: " + str(ret))
    print("R: " + str(R))
    print("T: " + str(T))

if __name__ == "__main__":
    main()