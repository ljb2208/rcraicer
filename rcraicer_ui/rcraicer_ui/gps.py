
class GPS():
    def __init__(self, UIQueue):
        self.satellites_visible = 0
        self.satellites_used = 0
        self.status = 0
        self.rtk_status  = 0
        self.block1_jamming = 0
        self.block1_jamming_indicator = 0
        self.block1_noise = 0
        self.block2_jamming = 0
        self.block2_jamming_indicator = 0
        self.block2_noise = 0

        self.UIQueue = UIQueue


        def gpsStatusCallback(self, msg):
            self.status = msg.status
            self.satellites_used = msg.satellites_used
            self.satellites_visible = msg.satellites_visible
            self.rtk_status = msg.rtk_status


        def gpsRFStatusCallback(self, msg):
            self.block1_jamming = msg.block1_jamming
            self.block1_jamming_indicator = msg.block1_jamming_ind
            self.block1_noise = msg.block1_noise

            self.block2_jamming = msg.block2_jamming
            self.block2_jamming_indicator = msg.block2_jamming_ind
            self.block2_noise = msg.block2_noise
    
    