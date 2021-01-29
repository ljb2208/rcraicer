import cv2
import numpy as np
import pyrealsense2 as rs

currentImage = None

class RSImage:
    def RSImage(self, colorImage, depthImage):
        self.colorImage = colorImage
        self.depthImage = depthImage

    def extractDepth(self, x, y):

        print("Depth: " + str(self.depthImage[y, x]))        
        return 0



def onClick(event,x,y,flags,param):    

    if event == cv2.EVENT_LBUTTONUP:
        if currentImage is not None:
            currentImage.extractDepth(x, y)

def main():
    global currentImage
    currentImage = RSImage()

    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    pipeline.start(config)

    cv2.namedWindow("Color")
    cv2.setMouseCallback("Color", onClick)    

    while (True):    

        rsframes = pipeline.wait_for_frames()
        color_frame = rsframes.get_color_frame()
        depth_frame = rsframes.get_depth_frame()
        
        currentImage.depthImage = np.asanyarray(depth_frame.get_data())
        currentImage.colorImage = np.asanyarray(color_frame.get_data())        

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(currentImage.depthImage, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((currentImage.colorImage, depth_colormap))


        cv2.imshow("Color", currentImage.colorImage)
        cv2.imshow("Depth", images)
        

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    pipeline.stop()
    cv2.destroyAllWindows()



if __name__ == "__main__":
    main()