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
    

def getCorners(image, cdict):        

    corners, markerIds, rejectedPoints = cv2.aruco.detectMarkers(image, cdict)

    return corners, markerIds
        


def loadData(cdict):
    index = 0
        
    leftCornerPoints = []
    rightCornerPoints = []
    leftIds = []
    rightIds = []
    
    filePath = str(pathlib.Path(__file__).parent.absolute())

    leftFilename =  filePath + "/charuco_images/left_" + str(index) + ".png"
    rightFilename = filePath + "/charuco_images/right_" + str(index) + ".png"

    while (path.exists(leftFilename) and path.exists(rightFilename)):
        leftImage = cv2.imread(leftFilename)
        rightImage = cv2.imread(rightFilename)

        lc, li = getCorners(leftImage, cdict)
        rc, ri = getCorners(rightImage, cdict)

        leftCornerPoints.append(np.float32(lc))
        leftIds.append(li)

        rightCornerPoints.append(np.float32(rc))
        rightIds.append(ri)

        index += 1

        print("Left corners: " + str(len(lc)) + " ids: " + str(len(li)))

        leftFilename =  filePath + "/charuco_images/left_" + str(index) + ".png"
        rightFilename = filePath + "/charuco_images/right_" + str(index) + ".png"

    print(str((index)) + " frames loaded.")    
    print(str(len(leftCornerPoints)) + " left corner points loaded.")
    print(str(len(rightCornerPoints)) + " right corner points loaded.")

    return leftCornerPoints, leftIds, rightCornerPoints, rightIds
    

def main():

    cdict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    board = cv2.aruco.CharucoBoard_create(	8, 6, 0.35, 0.2, cdict)
    params = cv2.aruco.DetectorParameters_create()


    leftCornerPoints, leftIds, rightCornerPoints, rightIds = loadData(cdict)

    # calibrate left camera
    # ret, lmtx, ldist, lnop = calibrateCamera(objectPoints, leftCornerPoints, True)
    ret, lmtx, ldist, lr, lt = cv2.aruco.calibrateCameraCharuco(leftCornerPoints, leftIds, board, (1024, 760), None, None)
    print("Left camera calibration returned: " + str(ret))

    # ret, rmtx, rdist, rr, rt = cv2.aruco.calibrateCameraCharuco(np.float32(rightCornerPoints), rightIds, board, (1024, 760), None, None)
    # print("Right camera calibration returned: " + str(ret))

    # ret, slmat, sldist, srmat, srdist, R, T, E, F = cv2.stereoCalibrate(objectPoints, leftCornerPoints, rightCornerPoints, lmtx, ldist, rmtx, rdist, (640, 480))

    # print("Stereo calibration returned: " + str(ret))
    # print("R: " + str(R))
    # print("T: " + str(T))

if __name__ == "__main__":
    main()