import cv2
import numpy as np
from stereo import Stereo
import math
import queue

class ObjectIsolator():
    def __init__(self, threshold=50):
        self.threshold = threshold
        self.minX = 10000
        self.maxX = -1
        self.minY = 10000
        self.maxY = -1

        self.searchMap = None
        self.searchQ = queue.Queue()
        self.iImage = None

    def isolateObject(self, image, x, y):        
        self.iImage = np.zeros_like(image)
        self.searchMap = np.zeros_like(image)

        initialVal = image[y][x]
        
        self.searchQ.put([x, y])

        while not self.searchQ.empty():
            search = self.searchQ.get()
            sX = search[0]
            sY = search[1]
            self.performSearch(sX, sY, initialVal, image)        
            

        return self.iImage    

    def performSearch(self, x, y, initialVal, image):

        if x < 0 or y < 0:
            return

        if x >= image.shape[1] or y >= image.shape[0]:
            return
        
        val = image[y][x]

        if self.searchMap[y][x] > 1:
            # already searched so return
            return
        
        if abs(int(val) - int(initialVal)) > self.threshold:
            # mark pixel as do not search further as edge detected
            self.searchMap[y][x] = 3             
            return        

        # mark pixel as searched
        self.searchMap[y][x] = 2
        self.iImage[y][x] = 255

        if x < self.minX:
            self.minX = x
        
        if x > self.maxX:
            self.maxX = x

        if y < self.minY:
            self.minY = y

        if y > self.maxY:
            self.maxY = y                        

        # check adjacent pixels
        #left
        newX = x - 1
        newY = y
        self.addToQueue(newX, newY)

        #right        
        newX = x + 1                
        self.addToQueue(newX, newY)

        #up
        newX = x
        newY = y + 1
        self.addToQueue(newX, newY)

        #down
        newY = y - 1
        self.addToQueue(newX, newY)

        return

    def addToQueue(self, x, y):
        if (self.searchMap[y][x] < 1):
            self.searchMap[y][x] = 1
            self.searchQ.put([x, y])            
    
    def calculateMeanDepth(self, image3d, stereo):

        meanDepth = [0., 0., 0.]
        sampleCount = 0


        for x in range(self.minX, self.maxX + 1):
            for y in range(self.minY, self.maxY + 1):
                if self.searchMap[y][x] != 2:
                    continue

                sampleCount += 1
                xRemap, yRemap = stereo.remapPoint(x, y, True)

                xRemap = int(xRemap)
                yRemap = int(yRemap)

                meanDepth[0] += image3d[yRemap][xRemap][0]
                meanDepth[1] += image3d[yRemap][xRemap][1]
                meanDepth[2] += image3d[yRemap][xRemap][2]


        meanDepth[0] /= float(sampleCount)
        meanDepth[1] /= float(sampleCount)
        meanDepth[2] /= float(sampleCount)

        self.calculateStdDevDepth(meanDepth, image3d, stereo)

        print("Mean depth: " + str(meanDepth) + " samples: " + str(sampleCount))

    def calculateStdDevDepth(self, meanDepth, image3d, stereo):

        stdDepth = [0., 0., 0.]
        sampleCount = 0

        for x in range(self.minX, self.maxX + 1):
            for y in range(self.minY, self.maxY + 1):
                if self.searchMap[y][x] != 2:
                    continue

                sampleCount += 1
                xRemap, yRemap = stereo.remapPoint(x, y, True)

                xRemap = int(xRemap)
                yRemap = int(yRemap)

                stdDepth[0] += math.pow((image3d[yRemap][xRemap][0] - meanDepth[0]), 2)
                stdDepth[1] += math.pow((image3d[yRemap][xRemap][1] - meanDepth[1]), 2)
                stdDepth[2] += math.pow((image3d[yRemap][xRemap][2] - meanDepth[2]), 2)

        
        print("Std dev pre: " + str(stdDepth))

        stdDepth[0] /= float(sampleCount)
        stdDepth[1] /= float(sampleCount)
        stdDepth[2] /= float(sampleCount)        

        stdDepth[0] = math.sqrt(stdDepth[0])
        stdDepth[1] = math.sqrt(stdDepth[1])
        stdDepth[2] = math.sqrt(stdDepth[2])

        print("Std dev: " + str(stdDepth))


class ImageTools():
    def __init__(self, filterFloor=155):
        self.filterFloor = filterFloor        
        self.stereo = Stereo()
        self.fixedLeft= None
        self.fixedRight = None

    def getRedFilterImage(self,image):    
        ccimage = self.getFilterImage(image, 2)
        return ccimage

    def getFilterImage(self, image, doubleChannel):
        dchann = image[:,:,doubleChannel]
        nchann1 = None
        nchann2 = None

        if doubleChannel == 0:
            nchann1 = image[:,:,1]
            nchann2 = image[:,:,2]
        elif doubleChannel == 1:
            nchann1 = image[:,:,0]
            nchann2 = image[:,:,2]
        else:
            nchann1 = image[:,:,0]
            nchann2 = image[:,:,1]       

        floor = np.full_like(dchann, 155, dtype=np.int16)
        cc = np.full_like(dchann, 0, dtype=np.int16)
        cc = np.add(dchann, dchann, dtype=np.int16)
        cc = np.subtract(cc, nchann1, dtype=np.int16)
        cc = np.subtract(cc, nchann1, dtype=np.int16)
        cc = np.subtract(cc, floor, dtype=np.int16)
        
        cc = np.clip(cc, 0, 100)
        ccimage = np.add(cc, floor).astype(np.uint8)

        return ccimage

    def calculateDepth(self, leftImage, rightImage, x, y):        
        # newLeftImage = cv2.cvtColor(leftImage, cv2.COLOR_GRAY2BGR)
        # newRightImage = cv2.cvtColor(rightImage, cv2.COLOR_GRAY2BGR)        
        # depth, image3d = self.stereo.computeStereo(newLeftImage, newRightImage)
        depth, image3d, self.fixedLeft, self.fixedRight = self.stereo.computeStereo(leftImage, rightImage)

        point3d = image3d[y][x]

        eucDist = math.sqrt(math.pow(point3d[0], 2) + math.pow(point3d[1], 2) + math.pow(point3d[2], 2))

        return point3d, eucDist

    # def isolateObject(self, leftImage, x, y):

    def detectEdges(self, image):
        imageBlur = cv2.blur(image, (3, 3))
        detectedEdges = cv2.Canny(imageBlur, 10, 10*3)

        mask = detectedEdges != 0
        imageEdge = image * (mask[:,:].astype(image.dtype))

        return imageEdge

    def getDepthImage(self, leftImage, rightImage):
        depth, image3d = self.stereo.computeStereo(leftImage, rightImage)
        return depth, image3d

    def getDepthImageMod(self, leftImage, rightImage):
        raw_disp, filtered_disp, image3d, self.fixedLeft, self.fixedRight = self.stereo.computeStereoMod(leftImage, rightImage)
        return raw_disp, filtered_disp, image3d

    def getDepthAtPoint(self, image3d, x, y):
        point3d = image3d[y][x]
        eucDist = math.sqrt(math.pow(point3d[0], 2) + math.pow(point3d[1], 2) + math.pow(point3d[2], 2))

        return point3d, eucDist

    def detectOrb(self, image, x, y):
        # image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        orb = cv2.ORB_create()                
        keypoints = orb.detect(image, None)
        keypoints, desc = orb.compute(image, keypoints)
        keypointsImage = cv2.drawKeypoints(image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        return keypointsImage

    def isolateObject(self, leftImage, rightImage, leftX, leftY, rightX, rightY):
        leftFilterImage = self.getRedFilterImage(leftImage)
        leftIsolator = ObjectIsolator()

        leftIImage = leftIsolator.isolateObject(leftFilterImage, leftX, leftY)                

        rightFilterImage = self.getRedFilterImage(rightImage)
        rightIsolator = ObjectIsolator()

        rightIImage = leftIsolator.isolateObject(rightFilterImage, rightX, rightY)                

        return leftIImage, rightIImage

    def getConeDepth(self, leftImage, rightImage, leftX, leftY, rightX, rightY):
        leftFilterImage = self.getRedFilterImage(leftImage)
        leftIsolator = ObjectIsolator()

        leftIImage = leftIsolator.isolateObject(leftFilterImage, leftX, leftY)                

        rightFilterImage = self.getRedFilterImage(rightImage)
        rightIsolator = ObjectIsolator()

        rightIImage = leftIsolator.isolateObject(rightFilterImage, rightX, rightY)                

        raw_disp, filtered_disp, image3d, self.fixedLeft, self.fixedRight = self.stereo.computeStereoMod(leftIImage, rightIImage)

        remapX, remapY = self.stereo.remapPoint(leftX, leftY, True)

        leftIsolator.calculateMeanDepth(image3d, self.stereo)

        return self.getDepthAtPoint(image3d, int(remapX), int(remapY))



        
            

