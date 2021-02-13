import cv2
import numpy as np
import pathlib
import pickle
from os import path
from ximea import xiapi
from datetime import datetime



def calibrateCamera(cornerPoints, ids, board, leftCamera):

    # TODO - Try calibrateCameraRO    
    ret, mtx, dist, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(cornerPoints, ids, board, (1280, 1024), None, None)

    filePath = str(pathlib.Path(__file__).parent.absolute())
    filePath += "/calibration/"

    if leftCamera == True:
        filePath += "left_charuco.pkl"
    else:
        filePath += "right_charuco.pkl"

    with open(filePath, 'wb') as handle:
        pickle.dump(mtx, handle)
        pickle.dump(dist, handle)
        pickle.dump(rvecs, handle)
        pickle.dump(tvecs, handle)

    return ret, mtx, dist
    

def getCorners(image, cdict, board):        
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    corners, markerIds, rejectedPoints = cv2.aruco.detectMarkers(gray, cdict)

    ret, ccorners, cids = cv2.aruco.interpolateCornersCharuco(corners, markerIds, gray, board)

    return ccorners, cids
        


def loadData(cdict, board):
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

        lc, li = getCorners(leftImage, cdict, board)
        rc, ri = getCorners(rightImage, cdict, board)

        leftCornerPoints.append(lc)
        leftIds.append(li)

        rightCornerPoints.append(rc)
        rightIds.append(ri)

        index += 1        

        leftFilename =  filePath + "/charuco_images/left_" + str(index) + ".png"
        rightFilename = filePath + "/charuco_images/right_" + str(index) + ".png"

    print(str((index)) + " frames loaded.")    
    print(str(len(leftCornerPoints)) + " left corner points loaded.")
    print(str(len(rightCornerPoints)) + " right corner points loaded.")

    return leftCornerPoints, leftIds, rightCornerPoints, rightIds
    

def main():

    cdict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
    board = cv2.aruco.CharucoBoard_create(	7, 5, 0.032, 0.016, cdict)
    params = cv2.aruco.DetectorParameters_create()

    leftCornerPoints, leftIds, rightCornerPoints, rightIds = loadData(cdict, board)
    
    ret, lmtx, ldist = calibrateCamera(leftCornerPoints, leftIds, board, True)
    print("Left camera calibration returned: " + str(ret))

    ret, rmtx, rdist = calibrateCamera(rightCornerPoints, rightIds, board, False)
    print("Right camera calibration returned: " + str(ret))

    # ret, slmat, sldist, srmat, srdist, R, T, E, F = cv2.stereoCalibrate(objectPoints, leftCornerPoints, rightCornerPoints, lmtx, ldist, rmtx, rdist, (640, 480))

    # print("Stereo calibration returned: " + str(ret))
    # print("R: " + str(R))
    # print("T: " + str(T))

if __name__ == "__main__":
    main()