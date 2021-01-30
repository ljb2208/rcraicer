#!/usr/bin/env python3.6

import cv2
import numpy as np
import pathlib
from os import path


def main():
    chardict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)    
    board = cv2.aruco.CharucoBoard_create(	8, 6, 0.35, 0.2, chardict)

    boardImage = board.draw((650, 550), 10, 1)

    cv2.imwrite("board.png", boardImage)

    



if __name__ == "__main__":
    main()