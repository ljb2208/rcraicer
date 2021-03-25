import cv2
import numpy as np
import pathlib
import pickle
import math
from os import path

class Stereo:
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
        self.leftMat = None
        self.leftDist = None
        self.rightMat = None
        self.rightDist = None

        self.maxDisp = 160
        self.windowSize = 3
        self.preFilterCap = 63
        self.p1 = 24
        self.p2 = 96
        self.wlsLambda = 8000.
        self.wlsSigma = 1.5
        self.fbsSpatial = 16.0
        self.fbsLuma = 8.0
        self.fbsChroma = 8.0
        self.fbsLambda = 128.0
        self.visMult = 1.0

        # load calibration data from pickle file
        self.loadConfig()        
        self.createStereoMatchers()

    def loadConfig(self):
        filePath = str(pathlib.Path(__file__).parent.absolute())
        filePath += "/../scratch/" + self.config + "/stereo.pkl"    

        with open(filePath, 'rb') as handle:
            self.leftMat = pickle.load(handle)
            self.leftDist = pickle.load(handle)
            self.rightMat = pickle.load(handle)
            self.rightDist = pickle.load(handle)
            self.R = pickle.load(handle)
            self.T = pickle.load(handle)
            # E = pickle.load(handle)
            # F = pickle.load(handle)

        print("Left mat: " + str(self.leftMat))
        print("Right mat: " + str(self.rightMat))
        print("R: " + str(self.R))
        print("T: " + str(self.T))

        leftRect, rightRect, leftProj, rightProj, self.Q, leftROI, rightROI = cv2.stereoRectify(self.leftMat, self.leftDist, self.rightMat, self.rightDist, self.imageSize, self.R, self.T) #, flags=cv2.CALIB_ZERO_DISPARITY
        self.leftMapX, self.leftMapY = cv2.initUndistortRectifyMap(self.leftMat, self.leftDist, leftRect, leftProj, self.imageSize, cv2.CV_32FC1)
        self.rightMapX, self.rightMapY = cv2.initUndistortRectifyMap(self.rightMat, self.rightDist, rightRect, rightProj, self.imageSize, cv2.CV_32FC1)

    def createStereoMatchers(self):
        p1Mod = self.p1 * self.windowSize * self.windowSize
        p2Mod = self.p2 * self.windowSize * self.windowSize

        self.leftMatcher = cv2.StereoSGBM_create(0, int(self.maxDisp), self.windowSize, p1Mod, p2Mod,preFilterCap=self.preFilterCap,mode=cv2.StereoSGBM_MODE_SGBM_3WAY)
        self.wlsFilter = cv2.ximgproc.createDisparityWLSFilter(self.leftMatcher)
        self.rightMatcher = cv2.ximgproc.createRightMatcher(self.leftMatcher)


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

    def remapPoint(self, x, y, isLeft):
        if isLeft:
            return self.leftMapX[y][x], self.leftMapY[y][x]
        else:
            return self.rightMapX[y][x], self.rightMapY[y][x]
    
    def computeStereo(self, leftImage, rightImage):
        fixedLeft = cv2.remap(leftImage, self.leftMapX, self.leftMapY, cv2.INTER_LINEAR)
        fixedRight = cv2.remap(rightImage, self.rightMapX, self.rightMapY, cv2.INTER_LINEAR)

        leftGray = cv2.cvtColor(fixedLeft, cv2.COLOR_BGR2GRAY)        
        rightGray = cv2.cvtColor(fixedRight, cv2.COLOR_BGR2GRAY)      

        depth = self.stereoMatcher.compute(leftGray, rightGray).astype(np.float32) / 16.0                
        image3d = cv2.reprojectImageTo3D(depth, self.Q)

        return depth, image3d, fixedLeft, fixedRight

    def computeStereoMod(self, leftImage, rightImage):        

        fixedLeft = cv2.remap(leftImage, self.leftMapX, self.leftMapY, cv2.INTER_LINEAR)
        fixedRight = cv2.remap(rightImage, self.rightMapX, self.rightMapY, cv2.INTER_LINEAR)        

        if len(leftImage.shape) > 2:
            leftImageForMatcher = cv2.cvtColor(fixedLeft, cv2.COLOR_BGR2GRAY)
            rightImageForMatcher = cv2.cvtColor(fixedRight, cv2.COLOR_BGR2GRAY)
        else:
            leftImageForMatcher = fixedLeft
            rightImageForMatcher = fixedRight

        left_disp = self.leftMatcher.compute(leftImageForMatcher, rightImageForMatcher)
        right_disp =  self.rightMatcher.compute(rightImageForMatcher, leftImageForMatcher)

        self.wlsFilter.setLambda(self.wlsLambda)
        self.wlsFilter.setSigmaColor(self.wlsSigma)

        filtered_disp = self.wlsFilter.filter(left_disp, leftImage, None, right_disp)
        # conf_map = wlsFilter.getConfidenceMap()

        raw_disp_viz = cv2.ximgproc.getDisparityVis(left_disp, self.visMult)
        filtered_disp_viz = cv2.ximgproc.getDisparityVis(filtered_disp, self.visMult)

        image3d = cv2.reprojectImageTo3D(filtered_disp_viz, self.Q)

        return raw_disp_viz, filtered_disp_viz, image3d, fixedLeft, fixedRight

        # solved_disp = cv2.ximgproc.fastBilateralSolverFilter(leftImage, left_disp, conf_map/255.0, None, fbs_spatial, fbs_luma, fbs_chroma, fbs_lambda)

        # solved_filtered_disp = cv2.ximgproc.fastBilateralSolverFilter(leftImage, filtered_disp, conf_map/255.0, None, fbs_spatial, fbs_luma, fbs_chroma, fbs_lambda)

        # solved_disp_viz = cv2.ximgproc.getDisparityVis(solved_disp, vis_mult)
        # solved_filtered_disp_viz = cv2.ximgproc.getDisparityVis(solved_filtered_disp, vis_mult)

        # return solved_disp_viz, solved_filtered_disp_viz

        