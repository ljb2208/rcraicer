

from ximea import xiapi

class Camera:
    def __init__(self, name, serial, fmt='XI_RGB24', downsampling='XI_DWN_2x2'):
        self.name = name
        self.serial = serial
        self.image = xiapi.Image()
        self.format = fmt
        self.downsampling = downsampling

    def open(self):
        self.cam = xiapi.Camera()
        self.cam.open_device_by_SN(self.serial)

        self.applySettings()
        self.cam.start_acquisition()

    def close(self):
        self.cam.stop_acquisition()
        self.cam.close_device()

    def applySettings(self):
        self.cam.set_downsampling(self.downsampling)    
        self.cam.set_imgdataformat(self.format)    
        self.cam.enable_aeag()    
        self.cam.enable_auto_wb()    

    def getImage(self):
        self.cam.get_image(self.image)
        return self.image.get_image_data_numpy()



    
