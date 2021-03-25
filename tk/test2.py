import cv2
import numpy as np
from stereo import Stereo
import math

if __name__ == '__main__':
    # rvec = np.array([-0.00773565, 0.53638681, 0.01040489], dtype='float32')
    # tvec = np.array([0.03577007, -0.0135745, -0.43895252], dtype='float32')

    # rvec = np.array([-0.01556469, 0.56776027, -0.01417326], dtype='float32')
    # tvec = np.array([-0.01393769, -0.04487603, -0.19031637], dtype='float32')

    pts = np.array([[10,5],[20,30],[70,20],[50,10]], np.int32)
    print(str(pts))
    pts = pts.reshape((-1,1,2))
    print(str(pts))

    img = cv2.imread('/home/lbarnett/ros2_ws/src/rcraicer/tk/images/left_0.png')
    cv2.polylines(img,[pts],True,(0,255,255))

    cv2.imshow("Left", img)
        
    cv2.waitKey()
        


    rvec = np.array([0.02342519, -0.27909857, 0.00258317], dtype='float32')
    tvec = np.array([-0.04465982, -0.01888669, -0.2570713], dtype='float32')


    # cone1 = np.array([[0.6244246],[0.59622145],	[1.8437463]], dtype='float32')
    cone1 = np.array([-0.4870041,0.5739608, 1.8282526], dtype='float32')

    rmat, jac = cv2.Rodrigues(rvec)
    
    # rmat[0).append(tvec[0])
    # rmat[1][3] = tvec[1]
    # rmat[2][3] = tvec[2]

    print("Rotation: " + str(rmat))

    # rmat = np.concatenate((rmat, tvec), axis=1)
    # print("Rotation 2: " + str(rmat))


    val = np.dot(cone1, rmat)
    # val = cone1 * rmat

    print("Val: " + str(val))

    val1 = val - tvec

    print("Val1: " + str(val1))
