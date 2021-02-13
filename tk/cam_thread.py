from queue import Queue
from ximea_cam import Camera


class CameraThread():
    def __init__(self, parent, messageQueue):        
        self.leftCamera = Camera("left", '31703351', downsampling="XI_DWN_1x1")
        self.rightCamera = Camera("right", '32703551', downsampling="XI_DWN_1x1")
        self.parent = parent
        self.running = True
        self.messageQueue = messageQueue

    def run(self):
        self.leftCamera.open()
        self.rightCamera.open()

        while (self.running):
            leftImage = self.leftCamera.getImage()
            rightImage = self.rightCamera.getImage()

            self.messageQueue.put([leftImage, rightImage])

        self.leftCamera.close()
        self.rightCamera.close()

