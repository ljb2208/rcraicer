import cv2
import numpy as np
import pathlib
import pickle
from os import path
from ximea_cam import Camera
from datetime import datetime   


def loadConfig():
    filePath = str(pathlib.Path(__file__).parent.absolute())
    filePath += "/calibration/stereo.pkl"    


    with open(filePath, 'rb') as handle:
        slmat = pickle.load(handle)
        sldist = pickle.load(handle)
        srmat = pickle.load(handle)
        srdist = pickle.load(handle)
        R = pickle.load(handle)
        T = pickle.load(handle)
        E = pickle.load(handle)
        F = pickle.load(handle)
    
    return slmat, sldist, srmat, srdist, R, T
    

def main():

    slmat, sldist, srmat, srdist, rmat, tvec = loadConfig()
    lrect, rrect, lproj, rproj, dtodm, leftROI, rightROI = cv2.stereoRectify(slmat, sldist, srmat, srdist, (1280, 1024), rmat, tvec)
    lmapx, lmapy = cv2.initUndistortRectifyMap(slmat, sldist, lrect, lproj, (1280, 1024), cv2.CV_32FC1)
    rmapx, rmapy = cv2.initUndistortRectifyMap(srmat, srdist, rrect, rproj, (1280, 1024), cv2.CV_32FC1)

    # stereoMatcher = cv2.StereoBM_create()

    # stereoMatcher.setMinDisparity(4)
    # stereoMatcher.setNumDisparities(128)
    # stereoMatcher.setBlockSize(21)
    # stereoMatcher.setSpeckleRange(16)
    # stereoMatcher.setSpeckleWindowSize(45)

    # stereoMatcher = cv2.StereoSGBM_create()

    # disparity range is tuned for 'aloe' image pair
    window_size = 3
    min_disp = 16
    num_disp = 112-min_disp
    stereoMatcher = cv2.StereoSGBM_create(minDisparity = min_disp,
        numDisparities = num_disp,
        blockSize = 16,
        P1 = 8*3*window_size**2,
        P2 = 32*3*window_size**2,
        disp12MaxDiff = 1,
        uniquenessRatio = 10,
        speckleWindowSize = 100,
        speckleRange = 32
    )

    leftCam = Camera("left", '31703351', downsampling="XI_DWN_1x1")
    rightCam = Camera("right", '32703551', downsampling="XI_DWN_1x1")

    leftCam.open()
    rightCam.open()

    cv2.namedWindow("Left")
    cv2.namedWindow("Right")

    while (True):                    
        
        leftData = leftCam.getImage()
        rightData = rightCam.getImage()        

        fixedLeft = cv2.remap(leftData, lmapx, lmapy, cv2.INTER_LINEAR)
        fixedRight = cv2.remap(rightData, rmapx, rmapy, cv2.INTER_LINEAR)

        lgray = cv2.cvtColor(fixedLeft, cv2.COLOR_BGR2GRAY)
        rgray = cv2.cvtColor(fixedRight, cv2.COLOR_BGR2GRAY)

        depth = stereoMatcher.compute(lgray, rgray)


        cv2.imshow("Left", leftData)
        cv2.imshow("Right", rightData)        
        cv2.imshow("Depth", depth / 2048)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    leftCam.close()
    rightCam.close()
    cv2.destroyAllWindows()



if __name__ == "__main__":
    main()