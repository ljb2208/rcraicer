import cv2
import numpy as np
import pathlib
import pickle
import math
from os import path

class StereoCamera:
    def __init__(self, config="calibration", imageSize=(1280,1024)):
        self.config = config
        self.imageSize = imageSize
        self.R = None
        self.T = None        
        self.Q = None
        self.leftMapX = None
        self.leftMapY = None
        self.rightMapX = None
        self.rightMapY = None
        self.stereoMatcher = None
        
        # load calibration data from pickle file
        self.loadConfig()        
        self.createStereoMatcher()

    def loadConfig(self):
        filePath = str(pathlib.Path(__file__).parent.absolute())
        filePath += "/" + self.config + "/stereo.pkl"    

        with open(filePath, 'rb') as handle:
            leftMat = pickle.load(handle)
            leftDist = pickle.load(handle)
            rightMat = pickle.load(handle)
            rightDist = pickle.load(handle)
            self.R = pickle.load(handle)
            self.T = pickle.load(handle)
            # E = pickle.load(handle)
            # F = pickle.load(handle)

        print("Left mat: " + str(leftMat))
        print("Right mat: " + str(rightMat))
        print("R: " + str(self.R))
        print("T: " + str(self.T))

        leftRect, rightRect, leftProj, rightProj, self.Q, leftROI, rightROI = cv2.stereoRectify(leftMat, leftDist, rightMat, rightDist, self.imageSize, self.R, self.T, flags=cv2.CALIB_ZERO_DISPARITY)
        self.leftMapX, self.leftMapY = cv2.initUndistortRectifyMap(leftMat, leftDist, leftRect, leftProj, self.imageSize, cv2.CV_32FC1)
        self.rightMapX, self.rightMapY = cv2.initUndistortRectifyMap(rightMat, rightDist, rightRect, rightProj, self.imageSize, cv2.CV_32FC1)

    def createStereoMatcher(self, windowSize=3, minDisp=16, blockSize=16, dispMaxDiff=1, uniquenessRatio=10, speckleWindowSize=100, speckleRange=32):                

        newMinDisp = int(round(minDisp/16)) * 16
        numDisp = 112-newMinDisp

        self.stereoMatcher = cv2.StereoSGBM_create(minDisparity = minDisp,
            numDisparities = numDisp,
            blockSize = blockSize,
            P1 = 8*3*windowSize**2,
            P2 = 32*3*windowSize**2,
            disp12MaxDiff = dispMaxDiff,
            uniquenessRatio = uniquenessRatio,
            speckleWindowSize = speckleWindowSize,
            speckleRange = speckleRange
        )
    
    def computeStereo(self, leftImage, rightImage):
        fixedLeft = cv2.remap(leftImage, self.leftMapX, self.leftMapY, cv2.INTER_LINEAR)
        fixedRight = cv2.remap(rightImage, self.rightMapX, self.rightMapY, cv2.INTER_LINEAR)

        leftGray = cv2.cvtColor(fixedLeft, cv2.COLOR_BGR2GRAY)        
        rightGray = cv2.cvtColor(fixedRight, cv2.COLOR_BGR2GRAY)      

        depth = self.stereoMatcher.compute(leftGray, rightGray)
        depthAmend = depth.astype(np.float32)
        depthAmend /= 16.
                
        image3d = cv2.reprojectImageTo3D(depthAmend, self.Q)

        return depth, image3d
