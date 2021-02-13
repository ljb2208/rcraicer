import cv2
import numpy as np
import pathlib
import pickle
from os import path
from ximea_cam import Camera
from stereo_cam import StereoCamera
from datetime import datetime   


xSel = 0
ySel = 0

xLeftSel = 0
yLeftSel = 0

xRightSel = 0
yRightSel = 0

captureCone = False
ciLeftCapture = None
ciRightCapture = None

windowSize=3
minDisp=16
blockSize=16
dispMaxDiff=1
uniquenessRatio=10
speckleWindowSize=100
speckleRange=32               
settingsChanged = False

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

def getCCRedImage(image):

    
    rchann = image[:,:,2]        
    gchann = image[:,:,1]
    bchann = image[:,:,0]

    floor = np.full_like(rchann, 155, dtype=np.int16)

    cc = np.full_like(rchann, 0, dtype=np.int16)
    cc = np.add(rchann, rchann, dtype=np.int16)
    cc = np.subtract(cc, gchann, dtype=np.int16)
    cc = np.subtract(cc, bchann, dtype=np.int16)
    cc = np.subtract(cc, floor, dtype=np.int16)
    
    cc = np.clip(cc, 0, 100)
    ccimage = np.add(cc, floor).astype(np.uint8)

    return ccimage

def getCCGreenImage(image):

    rchann = image[:,:,2]        
    gchann = image[:,:,1]
    bchann = image[:,:,0]

    floor = np.full_like(rchann, 155, dtype=np.int16)

    cc = np.full_like(rchann, 0, dtype=np.int16)
    cc = np.add(gchann, gchann, dtype=np.int16)
    cc = np.subtract(cc, rchann, dtype=np.int16)
    cc = np.subtract(cc, bchann, dtype=np.int16)
    cc = np.subtract(cc, floor, dtype=np.int16)
    
    cc = np.clip(cc, 0, 100)
    ccimage = np.add(cc, floor).astype(np.uint8)

    return ccimage

def getCCBlueImage(image):

    rchann = image[:,:,2]        
    gchann = image[:,:,1]
    bchann = image[:,:,0]

    floor = np.full_like(rchann, 155, dtype=np.int16)

    cc = np.full_like(rchann, 0, dtype=np.int16)
    cc = np.add(bchann, bchann, dtype=np.int16)
    cc = np.subtract(cc, rchann, dtype=np.int16)
    cc = np.subtract(cc, gchann, dtype=np.int16)
    cc = np.subtract(cc, floor, dtype=np.int16)
    
    cc = np.clip(cc, 0, 100)
    ccimage = np.add(cc, floor).astype(np.uint8)

    return ccimage


def onClick(event,x,y,flags,param):    
    global xSel
    global ySel

    if event == cv2.EVENT_LBUTTONUP:
        xSel = x    
        ySel = y

def onLeftConeClick(event,x,y,flags,param):    
    global xLeftSel
    global yLeftSel

    if event == cv2.EVENT_LBUTTONUP:
        xLeftSel = x    
        yLeftSel = y

def onRightConeClick(event,x,y,flags,param):    
    global xRightSel
    global yRightSel

    if event == cv2.EVENT_LBUTTONUP:
        xRightSel = x    
        yRightSel = y

def onConeButton(event,x,y,flags,param):
    global captureCone

    captureCone = True
    

def onMinDisp(value):
    global settingsChanged
    global minDisp

    minDisp = value
    settingsChanged = True

def onWindowSize(value):
    global settingsChanged
    global windowSize

    windowSize = value
    settingsChanged = True

def onBlockSize(value):
    global settingsChanged
    global blockSize    

    if (value % 2) == 0:
        return

    blockSize = value
    settingsChanged = True

def onMaxDiff(value):
    global settingsChanged
    global dispMaxDiff

    dispMaxDiff = value
    settingsChanged = True

def onUniqueRatio(value):
    global settingsChanged
    global uniquenessRatio
    
    uniquenessRatio = value
    settingsChanged = True

def onSpeckleWindow(value):
    global settingsChanged
    global speckleWindowSize

    speckleWindowSize = value    
    settingsChanged = True

def onSpeckleRange(value):
    global settingsChanged
    global speckleRange   

    speckleRange = value
    settingsChanged = True

def main():

    global windowSize
    global minDisp
    global blockSize
    global dispMaxDiff
    global uniquenessRatio
    global speckleWindowSize
    global speckleRange            
    global settingsChanged   
    global captureCone
    global xRightSel
    global xLeftSel
    global yRightSel
    global yLeftSel

    global ciLeftCapture
    global ciRightCapture

    stereoCam = StereoCamera()
    leftCam = Camera("left", '31703351', downsampling="XI_DWN_1x1")
    rightCam = Camera("right", '32703551', downsampling="XI_DWN_1x1")

    leftCam.open()
    rightCam.open()

    cv2.namedWindow("Left")
    cv2.setMouseCallback("Left", onClick)    
    cv2.namedWindow("Right")
    cv2.namedWindow("Settings")

    cv2.namedWindow("Cone Left")
    cv2.createButton("Settings", onConeButton, "Cone")
    cv2.setMouseCallback("Cone Left", onLeftConeClick)


    cv2.namedWindow("Cone Right")
    cv2.setMouseCallback("Cone Right", onRightConeClick)

    cv2.createTrackbar("Min Disp", "Settings", 16, 100, onMinDisp)
    cv2.createTrackbar("Window Size", "Settings", 3, 100, onWindowSize)
    cv2.createTrackbar("Block Size", "Settings", 16, 100, onBlockSize)
    cv2.createTrackbar("Max Diff", "Settings", 1, 100, onMaxDiff)
    cv2.createTrackbar("Unique Ratio", "Settings", 10, 50, onUniqueRatio)
    cv2.createTrackbar("Speckle Window", "Settings", 100, 300, onSpeckleWindow)
    cv2.createTrackbar("Speckle Range", "Settings", 32, 100, onSpeckleRange)
        

    while (True):                    
        
        leftData = leftCam.getImage()
        rightData = rightCam.getImage()        

        if settingsChanged:
            stereoCam.createStereoMatcher(windowSize, minDisp, blockSize, dispMaxDiff, uniquenessRatio, speckleWindowSize, speckleRange)
            settingsChanged = False

        # depth, image3d = stereoCam.computeStereo(leftData, rightData)
        

        # fixedLeft = cv2.remap(leftData, lmapx, lmapy, cv2.INTER_LINEAR)
        # fixedRight = cv2.remap(rightData, rmapx, rmapy, cv2.INTER_LINEAR)

        ciLeft = getCCRedImage(leftData)
        ciRight = getCCRedImage(rightData)
        # coneImageGreen = getCCGreenImage(leftData)
        # coneImageBlue = getCCBlueImage(leftData)

        if captureCone:
            if xLeftSel > 0 and yLeftSel > 0 and xRightSel > 0 and yRightSel > 0:
                captureCone()
                captureCone = False

        # text = ""
        # if (xSel > 0 and ySel > 0):
        #     r = leftData[ySel][xSel][2]
        #     g = leftData[ySel][xSel][1]
        #     b = leftData[ySel][xSel][0]
        #     text = "x/y: " + str(xSel) + "/" + str(ySel) + " rgb: " + str(r) + "/" + str(g) + "/" + str(b)
        #     text2 = "Depth1: "
        #     text3 = "Depth2: "

        #     d = (25.6 * 4.56) /  depth[ySel][xSel]
            
        #     if depth is not None:
        #         text2 += str(image3d[ySel][xSel])
        #         text3 += str(d) + ":" + str(depth[ySel][xSel])
        #     else:
        #         text2 += "Unknown"

        #     leftData = cv2.putText(leftData, text, (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 1,  (0, 0, 255), 2)
        #     leftData = cv2.putText(leftData, text2, (40, 80), cv2.FONT_HERSHEY_SIMPLEX, 1,  (0, 0, 255), 2)
        #     leftData = cv2.putText(leftData, text3, (40, 120), cv2.FONT_HERSHEY_SIMPLEX, 1,  (0, 0, 255), 2)
        

        if xLeftSel > 0 and yLeftSel > 0:
            ciLeft = cv2.cvtColor(ciLeft,  cv2.COLOR_GRAY2BGR)
            ciLeft = cv2.circle(ciLeft, (xLeftSel, yLeftSel), 5, (255,0,0))

        if xRightSel > 0 and yRightSel > 0:
            ciRight = cv2.cvtColor(ciRight,  cv2.COLOR_GRAY2BGR)
            ciRight = cv2.circle(ciRight, (xRightSel, yRightSel), 5, (255,0,0))

        cv2.imshow("Left", leftData)
        # cv2.imshow("Right", fixedRight)          
        cv2.imshow("Cone Left", ciLeft)
        cv2.imshow("Cone Right", ciRight)

        # cv2.imshow("Depth", depth / 2048)        
        # cv2.imshow("CCGreen", coneImageGreen)
        # cv2.imshow("CCBlue", coneImageBlue)


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    leftCam.close()
    rightCam.close()
    cv2.destroyAllWindows()



if __name__ == "__main__":
    main()