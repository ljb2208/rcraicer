#!/usr/bin/env python3.6

import cv2
import numpy as np
import pathlib
from os import path


def main():
    retval = cv2.aruco.CharucoBoard_create(	11, 8, 0.15, 0.05, cv2.aruco.DICT4X4	)



if __name__ == "__main__":
    main()