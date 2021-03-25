import cv2
import numpy as np
from stereo import Stereo
import math

class PoseEstimator():
    def __init__(self):
        self.x = 0        
        self.stereo = Stereo()
        self.matches = None
        self.keyPoints1 = None
        self.desc1 = None
        self.keyPoints2 = None
        self.desc2 = None

        self.imagePoints = None
        self.modelPoints = None


    def calculatePose(self, leftImage, rightImage, leftImageNew, rightImageNew):
        self.getKeyPointsSIFT(leftImage, leftImageNew)
        raw_disp_viz, filtered_disp_viz, image3d, fixedLeft, fixedRight = self.stereo.computeStereoMod(leftImage, rightImage)
        self.mapImagePointsToModelPoints(leftImageNew, image3d)

        imgPoints = np.array(self.imagePoints, dtype='float32')
        modPoints = np.array(self.modelPoints, dtype='float32')

        success, rotation, translation, inliers = cv2.solvePnPRansac(modPoints, imgPoints, self.stereo.leftMat, self.stereo.leftDist)

        print("Rotation: " + str(rotation))
        print("Translation: " + str(translation))
        print("Success: " + str(success))
        

    def mapImagePointsToModelPoints(self, image, image3d):
        x = 0
        self.imagePoints = []
        self.modelPoints = []

        maxPoints = 10000
        numPoints = 0

        for match in self.matches:
            x = 0

            if numPoints > maxPoints:
                return

            kp1 = self.keyPoints1[match.queryIdx]
            kp2 = self.keyPoints2[match.trainIdx]
            
            self.imagePoints.append(kp2.pt)
            self.modelPoints.append(self.getPoint3d(kp1.pt, image3d))

            numPoints += 1

    
    def getPoint3d(self, pt, image3d):
        xRemap, yRemap = self.stereo.remapPoint(int(pt[0]), int(pt[1]), True)
        return image3d[int(yRemap)][int(xRemap)]


    def getKeyPoints(self, image1, image2):
        fixed1 = cv2.remap(image1, self.stereo.leftMapX, self.stereo.leftMapY, cv2.INTER_LINEAR)
        fixed2 = cv2.remap(image2, self.stereo.rightMapX, self.stereo.rightMapY, cv2.INTER_LINEAR)        
        grey1 = cv2.cvtColor(fixed1, cv2.COLOR_BGR2GRAY)
        grey2 = cv2.cvtColor(fixed2, cv2.COLOR_BGR2GRAY)

        # sift = cv2.SIFT_create()
        orb = cv2.ORB_create()

        self.keyPoints1, self.desc1 = orb.detectAndCompute(grey1, None)
        self.keyPoints2, self.desc2 = orb.detectAndCompute(grey2, None)

        # bf = cv2.BFMatcher()        
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.matches = bf.match(self.desc1,self.desc2)        

    def getKeyPointsORB(self, image1, image2):
        fixed1 = cv2.remap(image1, self.stereo.leftMapX, self.stereo.leftMapY, cv2.INTER_LINEAR)
        fixed2 = cv2.remap(image2, self.stereo.rightMapX, self.stereo.rightMapY, cv2.INTER_LINEAR)        
        grey1 = cv2.cvtColor(fixed1, cv2.COLOR_BGR2GRAY)
        grey2 = cv2.cvtColor(fixed2, cv2.COLOR_BGR2GRAY)

        # sift = cv2.SIFT_create()
        orb = cv2.ORB_create()

        self.keyPoints1, self.desc1 = orb.detectAndCompute(grey1, None)
        self.keyPoints2, self.desc2 = orb.detectAndCompute(grey2, None)

        # bf = cv2.BFMatcher()        
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.matches = bf.match(self.desc1,self.desc2)        

    def getKeyPointsSIFT(self, image1, image2):
        fixed1 = cv2.remap(image1, self.stereo.leftMapX, self.stereo.leftMapY, cv2.INTER_LINEAR)
        fixed2 = cv2.remap(image2, self.stereo.rightMapX, self.stereo.rightMapY, cv2.INTER_LINEAR)        
        grey1 = cv2.cvtColor(fixed1, cv2.COLOR_BGR2GRAY)
        grey2 = cv2.cvtColor(fixed2, cv2.COLOR_BGR2GRAY)

        sift = cv2.SIFT_create()        

        self.keyPoints1, self.desc1 = sift.detectAndCompute(grey1, None)
        self.keyPoints2, self.desc2 = sift.detectAndCompute(grey2, None)

        bf = cv2.BFMatcher()        
        matches = bf.knnMatch(self.desc1, self.desc2,k=2)                

        # Apply ratio test
        self.matches = []
        for m,n in matches:
            if m.distance < 0.75*n.distance:
                self.matches.append(m)
        
    
