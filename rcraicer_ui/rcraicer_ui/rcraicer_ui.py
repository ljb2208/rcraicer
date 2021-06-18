import rclpy
import queue
import threading
import sys

from tkinter import *
from tkinter import ttk

from rclpy.node import Node
from rcraicer_msgs.msg import GPSRFStatus, GPSStatus, GPSSurvey
from sensor_msgs.msg import NavSatFix


HEADER_LABEL = "Arial 14 bold"
FONT_LABEL = "Arial 11 bold"
FONT_NORMAL = "Arial 11"

class RCRaicerUI():

    def __init__(self, UIQueue):
        self.main = Tk()
        self.UIQueue = UIQueue 
        self.running = True       

        self.dopLevels = (2., 5.)        
        self.satUsedLevels = (20, 15)
        self.satVisibleLevels = (30, 25)
        self.jammingLevels = (15., 30.)
        self.noiseLevels = (15., 30.)
        self.obsLevels = (120, 60)
        self.surveyAccLevels = (2., 3.)
        self.accLevels = (0.05, 1.)
        self.meanLevels = (2.0, 3.0)
        self.rtkLevels = ("Float", "Fixed")

        self.main.title("GPS Status")
        self.main.protocol("WM_DELETE_WINDOW", self.close)   

        self.buildControls()
    
    def runMainLoop(self):        
        if self.running:
            self.main.update_idletasks()            
            self.main.update()                  
            self.processMessages()                  
            return True
        else:
            return False        


    def close(self):        
        self.running = False

    def exitApp(self):
        self.main.destroy()

    def processMessages(self):
        if not self.running:
            return

        while not self.UIQueue.empty():
            msg = self.UIQueue.get()

            if msg[0] == "rover":
                self.processRoverMessage(msg[1])
            elif msg[0] == "base":
                self.processBaseMessage(msg[1])

    def processRoverMessage(self, msg):        

        if type(msg) is GPSStatus:
            self.updateNumericValue(self.rhdopValue, "{:.3f}".format(msg.hdop), self.dopLevels)            
            self.updateNumericValue(self.rpdopValue, "{:.3f}".format(msg.pdop), self.dopLevels)
            self.updateNumericValue(self.rvdopValue, "{:.3f}".format(msg.vdop), self.dopLevels)
            self.updateNumericValue(self.rsuValue, msg.satellites_used, self.satUsedLevels)
            self.updateNumericValue(self.rsvValue, msg.satellites_visible, self.satVisibleLevels)
            self.updateTextValue(self.rrtkStatusValue, self.getRTKStatus(msg.rtk_status), self.rtkLevels)
            self.updateTextValue(self.rstatusValue, self.getGPSStatus(msg.status), None)
            self.updateNumericValue(self.rhaccValue, "{:.2f}".format(msg.hacc), self.accLevels)
            self.updateNumericValue(self.rvaccValue, "{:.2f}".format(msg.vacc), self.accLevels)
            self.updateNumericValue(self.rspeedValue, "{:.2f}".format(msg.gspeed), None)
            self.updateNumericValue(self.rspeedAccValue, "{:.2f}".format(msg.sacc), self.accLevels)
            self.updateNumericValue(self.rheadingValue, "{:.2f}".format(msg.headmot), None)
            self.updateNumericValue(self.rheadingAccValue, "{:.2f}".format(msg.headacc), self.accLevels)

        elif type(msg) is GPSRFStatus:
            self.updateTextValue(self.rjammingStatusValue1, self.getJammingStatus(msg.block1_jamming), None)
            self.updateTextValue(self.rjammingStatusValue2, self.getJammingStatus(msg.block2_jamming), None)
            self.updateNumericValue(self.rjammingIndValue1, "{:.1f}".format(float(msg.block1_jamming_ind) /2.55), self.jammingLevels)
            self.updateNumericValue(self.rjammingIndValue2, "{:.1f}".format(float(msg.block2_jamming_ind) /2.55), self.jammingLevels)
            self.updateNumericValue(self.rnoiseValue1, "{:.1f}".format(float(msg.block1_noise) /2.55), self.noiseLevels)
            self.updateNumericValue(self.rnoiseValue2, "{:.1f}".format(float(msg.block2_noise) /2.55), self.noiseLevels)

        elif type(msg) is NavSatFix:
            self.updateNumericValue(self.rlatValue, msg.latitude, None)
            self.updateNumericValue(self.rlonValue, msg.longitude, None)
            self.updateNumericValue(self.raltValue, msg.altitude, None)

    def processBaseMessage(self, msg):        
        if type(msg) is GPSStatus:
            self.updateNumericValue(self.bhdopValue, "{:.3f}".format(msg.hdop), self.dopLevels)            
            self.updateNumericValue(self.bpdopValue, "{:.3f}".format(msg.pdop), self.dopLevels)
            self.updateNumericValue(self.bvdopValue, "{:.3f}".format(msg.vdop), self.dopLevels)
            self.updateNumericValue(self.bsuValue, msg.satellites_used, self.satUsedLevels)
            self.updateNumericValue(self.bsvValue, msg.satellites_visible, self.satVisibleLevels)            
            self.updateTextValue(self.bstatusValue, self.getGPSStatus(msg.status), None)
            self.updateNumericValue(self.bhaccValue, "{:.2f}".format(msg.hacc), self.accLevels)
            self.updateNumericValue(self.bvaccValue, "{:.2f}".format(msg.vacc), self.accLevels)
            self.updateNumericValue(self.bspeedValue, "{:.2f}".format(msg.gspeed), None)
            self.updateNumericValue(self.bspeedAccValue, "{:.2f}".format(msg.sacc), self.accLevels)
            self.updateNumericValue(self.bheadingValue, "{:.2f}".format(msg.headmot), None)
            self.updateNumericValue(self.bheadingAccValue, "{:.2f}".format(msg.headacc), self.accLevels)


        elif type(msg) is GPSRFStatus:
            self.updateTextValue(self.bjammingStatusValue1, self.getJammingStatus(msg.block1_jamming), None)
            self.updateTextValue(self.bjammingStatusValue2, self.getJammingStatus(msg.block2_jamming), None)
            self.updateNumericValue(self.bjammingIndValue1, "{:.1f}".format(float(msg.block1_jamming_ind) /2.55), self.jammingLevels)
            self.updateNumericValue(self.bjammingIndValue2, "{:.1f}".format(float(msg.block2_jamming_ind) /2.55), self.jammingLevels)
            self.updateNumericValue(self.bnoiseValue1, "{:.1f}".format(float(msg.block1_noise) /2.55), self.noiseLevels)
            self.updateNumericValue(self.bnoiseValue2, "{:.1f}".format(float(msg.block2_noise) /2.55), self.noiseLevels)

        elif type(msg) is NavSatFix:
            self.updateNumericValue(self.blatValue, msg.latitude, None)
            self.updateNumericValue(self.blonValue, msg.longitude, None)
            self.updateNumericValue(self.baltValue, msg.altitude, None)

        elif type(msg) is GPSSurvey:
            self.updateNumericValue(self.bsurveyObsValue, msg.observations, self.obsLevels)
            self.updateNumericValue(self.bsurveyDurationValue, msg.duration, self.obsLevels)
            self.updateNumericValue(self.bsurveyAccuracyValue, "{:.3f}".format(msg.accuracy), self.surveyAccLevels)            
            self.updateTextValue(self.bsurveyStatusValue, self.getSurveyActive(msg.active), None)
            self.updateTextValue(self.bsurveyValidValue, self.getSurveyValid(msg.valid), None)
            self.updateNumericValue(self.bsurveyMeanXValue, "{:.3f}".format(msg.mean_x), self.meanLevels)            
            self.updateNumericValue(self.bsurveyMeanYValue, "{:.3f}".format(msg.mean_y), self.meanLevels)            
            self.updateNumericValue(self.bsurveyMeanZValue, "{:.3f}".format(msg.mean_z), self.meanLevels)            

    
    def getSurveyActive(self, statusVal):
        if statusVal == 1:
            return ("Active", 0)
        else:
            return ("Inact.", 2)

    def getSurveyValid(self, statusVal):
        if statusVal == 1:
            return ("Yes", 2)
        else:
            return ("No", 0)
        


    def getRTKStatus(self, statusVal):
        if statusVal == 0:
            return "None"
        elif statusVal == 1:
            return "Float"
        elif statusVal == 2:
            return "Fixed"

        return "Unknown"


    def getGPSStatus(self, statusVal):
        if statusVal == -1:
            return ("No Fix", 0)
        elif statusVal == 0:
            return ("Fix", 1)
        elif statusVal == 1:
            return ("SBAS", 1)
        elif statusVal == 2:
            return ("GBAS", 1)
        elif statusVal == 18:
            return ("DGPS", 2)
        elif statusVal == 33:
            return ("WAAS", 2)


        return ("Unknown", 0)

    def getJammingStatus(self, statusVal):        
            
        if statusVal == 1:
            return ("OK", 2)
        elif statusVal == 2:
            return ("WARN", 1)
        elif statusVal == 3:
            return ("CRIT", 2)

        return ("Unknown", 0)


    # def processBaseMessage(self, msg):
    #     print("base message")


    def updateNumericValue(self, ctrl, value, valueLevels):
        ctrl["text"]= str(value)

        color = "green"

        valueAmber = 0.
        valueRed = 0.

        if valueLevels is not None:
            valueAmber = valueLevels[0]
            valueRed = valueLevels[1]

            if type(valueLevels[0]) is float:
                value = float(value)
            else:
                value = int(value)

            if valueAmber > valueRed:
                if value < valueRed:
                    color = "red"
                elif value < valueAmber:
                    color = "orange"
            else:
                if value > valueRed:
                    color = "red"
                elif value > valueAmber:
                    color = "orange"

        ctrl["bg"] = color

    def updateTextValue(self, ctrl, value, valueLevels):

        level = -1

        if type(value) is tuple:
            ctrl["text"]= value[0]
            level = value[1]
        else:
            ctrl["text"]= value

        color = "red"        

        valueAmber = ""
        valueGreen = ""

        if valueLevels is not None:
            valueAmber = valueLevels[0]
            valueGreen = valueLevels[1]

        if level ==2 or value == valueGreen:
            color = "green"
        elif level ==1 or value == valueAmber:
            color = "orange"                

        ctrl["bg"] = color


    def buildControls(self):        

        rowId = 0
        rover = Label(self.main, text="Rover", font=HEADER_LABEL)
        rover.grid(row=rowId, column=0)

        base = Label(self.main, text="Base", font=HEADER_LABEL)
        base.grid(row=rowId, column=3)

        rowId += 1

        rhdopLabel = Label(self.main, text="HDOP", font=FONT_LABEL)
        rhdopLabel.grid(row=rowId, column=0)
        self.rhdopValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.rhdopValue.grid(row=rowId, column=1)    

        rowId += 1

        rpdopLabel = Label(self.main, text="PDOP", font=FONT_LABEL)
        rpdopLabel.grid(row=rowId, column=0)
        self.rpdopValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.rpdopValue.grid(row=rowId, column=1)    
        
        rowId += 1

        rvdopLabel = Label(self.main, text="VDOP", font=FONT_LABEL)
        rvdopLabel.grid(row=rowId, column=0)
        self.rvdopValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.rvdopValue.grid(row=rowId, column=1)    

        rowId += 1

        raccLabel = Label(self.main, text="Acc (Hor/Ver)", font=FONT_LABEL)
        raccLabel.grid(row=rowId, column=0)
        self.rhaccValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.rhaccValue.grid(row=rowId, column=1)
        self.rvaccValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.rvaccValue.grid(row=rowId, column=2)    
        
        rowId += 1

        rspeedLabel = Label(self.main, text="Speed/Acc", font=FONT_LABEL)
        rspeedLabel.grid(row=rowId, column=0)
        self.rspeedValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.rspeedValue.grid(row=rowId, column=1)
        self.rspeedAccValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.rspeedAccValue.grid(row=rowId, column=2)    
        
        rowId += 1

        rheadingLabel = Label(self.main, text="Heading/Acc", font=FONT_LABEL)
        rheadingLabel.grid(row=rowId, column=0)
        self.rheadingValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.rheadingValue.grid(row=rowId, column=1)
        self.rheadingAccValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.rheadingAccValue.grid(row=rowId, column=2)    
        
        rowId += 1

        rlatLabel = Label(self.main, text="Lat/Lon", font=FONT_LABEL)
        rlatLabel.grid(row=rowId, column=0)
        self.rlatValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=16)                                
        self.rlatValue.grid(row=rowId, column=1)    
        self.rlonValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=16)                                
        self.rlonValue.grid(row=rowId, column=2)    

        rowId += 1

        raltLabel = Label(self.main, text="Altitude", font=FONT_LABEL)
        raltLabel.grid(row=rowId, column=0)
        self.raltValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.raltValue.grid(row=rowId, column=1)    

        rowId += 1


        rsuLabel = Label(self.main, text="Satelittes Used/Vis", font=FONT_LABEL)
        rsuLabel.grid(row=rowId, column=0)
        self.rsuValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=16)                                
        self.rsuValue.grid(row=rowId, column=1)    
        self.rsvValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=16)                                
        self.rsvValue.grid(row=rowId, column=2)    

        rowId += 1

        rstatusLabel = Label(self.main, text="Status", font=FONT_LABEL)
        rstatusLabel.grid(row=rowId, column=0)
        self.rstatusValue = Label(self.main, text="Unknown", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.rstatusValue.grid(row=rowId, column=1)    

        rowId += 1

        rrtkStatusLabel = Label(self.main, text="RTK Status", font=FONT_LABEL)
        rrtkStatusLabel.grid(row=rowId, column=0)
        self.rrtkStatusValue = Label(self.main, text="None", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.rrtkStatusValue.grid(row=rowId, column=1)    

        rowId += 1

        rjammingStatusLabel = Label(self.main, text="Jamming Status", font=FONT_LABEL)
        rjammingStatusLabel.grid(row=rowId, column=0)
        self.rjammingStatusValue1 = Label(self.main, text="Unknown", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.rjammingStatusValue1.grid(row=rowId, column=1)    
        self.rjammingStatusValue2 = Label(self.main, text="Unknown", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.rjammingStatusValue2.grid(row=rowId, column=2)    

        rowId += 1


        rjammingIndLabel = Label(self.main, text="Jamming Ind.", font=FONT_LABEL)
        rjammingIndLabel.grid(row=rowId, column=0)
        self.rjammingIndValue1 = Label(self.main, text="Unknown", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.rjammingIndValue1.grid(row=rowId, column=1)    
        self.rjammingIndValue2 = Label(self.main, text="Unknown", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.rjammingIndValue2.grid(row=rowId, column=2)    

        rowId += 1

        rnoiseLabel = Label(self.main, text="Noise", font=FONT_LABEL)
        rnoiseLabel.grid(row=rowId, column=0)
        self.rnoiseValue1 = Label(self.main, text="0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.rnoiseValue1.grid(row=rowId, column=1)    
        self.rnoiseValue2 = Label(self.main, text="0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.rnoiseValue2.grid(row=rowId, column=2)            

        # base 
        rowId = 1
        
        self.bhdopValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bhdopValue.grid(row=rowId, column=3)    

        rowId += 1
        
        self.bpdopValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bpdopValue.grid(row=rowId, column=3)    
        
        rowId += 1
        
        self.bvdopValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bvdopValue.grid(row=rowId, column=3)    

        rowId += 1

        self.bhaccValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bhaccValue.grid(row=rowId, column=3)
        self.bvaccValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bvaccValue.grid(row=rowId, column=4)    
        
        rowId += 1

        self.bspeedValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bspeedValue.grid(row=rowId, column=3)
        self.bspeedAccValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bspeedAccValue.grid(row=rowId, column=4)    
        
        rowId += 1

        self.bheadingValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bheadingValue.grid(row=rowId, column=3)
        self.bheadingAccValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bheadingAccValue.grid(row=rowId, column=4)    
        
        rowId += 1
        
        self.blatValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.blatValue.grid(row=rowId, column=3)    
        self.blonValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.blonValue.grid(row=rowId, column=4)    

        rowId += 1
        
        self.baltValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.baltValue.grid(row=rowId, column=3)    

        rowId += 1
        
        self.bsuValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bsuValue.grid(row=rowId, column=3)    
        self.bsvValue = Label(self.main, text="0.0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bsvValue.grid(row=rowId, column=4)    

        rowId += 1
        
        self.bstatusValue = Label(self.main, text="Unknown", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bstatusValue.grid(row=rowId, column=3)    

        rowId += 1
        
        # skip rtk status for base

        rowId += 1
        
        self.bjammingStatusValue1 = Label(self.main, text="Unknown", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bjammingStatusValue1.grid(row=rowId, column=3)    
        self.bjammingStatusValue2 = Label(self.main, text="Unknown", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bjammingStatusValue2.grid(row=rowId, column=4)    

        rowId += 1
        
        self.bjammingIndValue1 = Label(self.main, text="Unknown", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bjammingIndValue1.grid(row=rowId, column=3)    
        self.bjammingIndValue2 = Label(self.main, text="Unknown", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bjammingIndValue2.grid(row=rowId, column=4)    

        rowId += 1
        
        self.bnoiseValue1 = Label(self.main, text="0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bnoiseValue1.grid(row=rowId, column=3)    
        self.bnoiseValue2 = Label(self.main, text="0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bnoiseValue2.grid(row=rowId, column=4)    

        rowId += 1

        bSurveyStatusLabel = Label(self.main, text="Survey Status", font=FONT_LABEL)
        bSurveyStatusLabel.grid(row=rowId, column=0)
        self.bsurveyStatusValue = Label(self.main, text="None", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bsurveyStatusValue.grid(row=rowId, column=3)    

        rowId += 1

        bSurveyValidLabel = Label(self.main, text="Survey Valid", font=FONT_LABEL)
        bSurveyValidLabel.grid(row=rowId, column=0)
        self.bsurveyValidValue = Label(self.main, text="None", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bsurveyValidValue.grid(row=rowId, column=3)    

        rowId += 1

        bSurveyObsLabel = Label(self.main, text="# Survey Obs", font=FONT_LABEL)
        bSurveyObsLabel.grid(row=rowId, column=0)
        self.bsurveyObsValue = Label(self.main, text="0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bsurveyObsValue.grid(row=rowId, column=3)    

        rowId += 1

        bSurveyDurationLabel = Label(self.main, text="Survey Dur.", font=FONT_LABEL)
        bSurveyDurationLabel.grid(row=rowId, column=0)
        self.bsurveyDurationValue = Label(self.main, text="0", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bsurveyDurationValue.grid(row=rowId, column=3)    

        rowId += 1

        bSurveyAccuracyLabel = Label(self.main, text="Survey Acc.", font=FONT_LABEL)
        bSurveyAccuracyLabel.grid(row=rowId, column=0)
        self.bsurveyAccuracyValue = Label(self.main, text="None", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bsurveyAccuracyValue.grid(row=rowId, column=3)    

        rowId += 1

        bSurveyMeanLabel = Label(self.main, text="Mean Vals", font=FONT_LABEL)
        bSurveyMeanLabel.grid(row=rowId, column=0)
        self.bsurveyMeanXValue = Label(self.main, text="None", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bsurveyMeanXValue.grid(row=rowId, column=3)    
        self.bsurveyMeanYValue = Label(self.main, text="None", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bsurveyMeanYValue.grid(row=rowId, column=4)    
        self.bsurveyMeanZValue = Label(self.main, text="None", borderwidth=2, relief="groove", bg="grey", width=8)                                
        self.bsurveyMeanZValue.grid(row=rowId, column=5)    

        rowId += 1

    

class RosThread(Node):
    def __init__(self, UIQueue):
        super().__init__("rcraicer_ui")

        self.running = True
        self.UIQueue = UIQueue

        self.roverStatusSub = self.create_subscription(GPSStatus, 
                                    'rover_gps_status',
                                    self.roverGpsStatusCallback,
                                    10)

        self.roverRFStatusSub = self.create_subscription(GPSRFStatus, 
                                    'rover_gps_rf_status',
                                    self.roverGpsRFStatusCallback,
                                    10)

        self.roverNavSatSub = self.create_subscription(NavSatFix, 
                                    'rover_navsat_fix',
                                    self.roverNavSatFixCallback,
                                    10)

        self.baseStatusSub = self.create_subscription(GPSStatus, 
                                    'base_gps_status',
                                    self.baseGpsStatusCallback,
                                    10)

        self.baseRFStatusSub = self.create_subscription(GPSRFStatus, 
                                    'base_gps_rf_status',
                                    self.baseGpsRFStatusCallback,
                                    10)


        self.baseNavSatSub = self.create_subscription(NavSatFix, 
                                    'base_navsat_fix',
                                    self.baseNavSatFixCallback,
                                    10)

        self.baseSurveySub = self.create_subscription(GPSSurvey, 
                                    'base_gps_survey',
                                    self.baseGpsSurveyCallback,
                                    10)

    def roverGpsStatusCallback(self, msg):
        self.queueMessage("rover", msg)

    def roverGpsRFStatusCallback(self, msg):
        self.queueMessage("rover", msg)

    def roverNavSatFixCallback(self, msg):
        self.queueMessage("rover", msg)

    def baseGpsStatusCallback(self, msg):
        self.queueMessage("base", msg)

    def baseGpsRFStatusCallback(self, msg):
        self.queueMessage("base", msg)

    def baseNavSatFixCallback(self, msg):
        self.queueMessage("base", msg)

    def baseGpsSurveyCallback(self, msg):
        self.queueMessage("base", msg)

    def queueMessage(self, target, msg):
        self.UIQueue.put([target, msg])

    def stop(self):        
        self.running = False        

global rosThreadObj

def runROSThread(UIQueue, args):
    global rosThreadObj

    rclpy.init(args=args)

    rosThreadObj = RosThread(UIQueue)    

    while rosThreadObj.running == True:
        rclpy.spin_once(rosThreadObj, timeout_sec=0.1)

    rosThreadObj.destroy_node()
    rclpy.shutdown()

    print("ROS thread exiting")
   
    


if __name__ == '__main__':	            
    global rosThreadObj
    UIQueue = queue.Queue()

    # start ros threa
    rosThread = threading.Thread(target=runROSThread, args=(UIQueue, sys.argv))
    rosThread.start()

    ui = RCRaicerUI(UIQueue)

    running = True
    
    while running:
        running = ui.runMainLoop()
        
        if running:
            running = rosThread.is_alive()
        
        
    rosThreadObj.stop()
    ui.running = False
    ui.exitApp()    

    print("exiting")