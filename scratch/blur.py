#!/usr/bin/env python3.6

import cv2
import numpy as np
import pathlib
from os import path


def main():
    
    index = 0
    filePath = str(pathlib.Path(__file__).parent.absolute())

    leftFilename =  filePath + "/images/left_" + str(index) + ".png"
    rightFilename = filePath + "/images/right_" + str(index) + ".png"

    while (path.exists(leftFilename) and path.exists(rightFilename)):
        leftImage = cv2.imread(leftFilename)
        rightImage = cv2.imread(rightFilename)

        leftGray = cv2.cvtColor(leftImage, cv2.COLOR_BGR2GRAY)
        rightGray = cv2.cvtColor(rightImage, cv2.COLOR_BGR2GRAY)

        print("Index: " + str(index))
        val = cv2.Laplacian(leftGray, cv2.CV_64F).var()
        print("Left: " + str(val))

        val = cv2.Laplacian(rightGray, cv2.CV_64F).var()
        print("Right: " + str(val))

        index += 1
        leftFilename =  filePath + "/images/left_" + str(index) + ".png"
        rightFilename = filePath + "/images/right_" + str(index) + ".png"



if __name__ == "__main__":
    main()