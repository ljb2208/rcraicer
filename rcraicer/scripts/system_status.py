#!/usr/bin/env python3
# Software License Agreement (BSD License)
# Copyright (c) 2013, Georgia Institute of Technology
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## @package systemStatus
#  Gets the wireless signal strength, power status, and CPU temperature. This information is published in the form of ROS messages.
#

#import roslib; roslib.load_manifest('autorally_msgs')
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#import commands
import subprocess
import signal
import sys
import math
import socket

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# This shared library is only importable when a GPU is installed
# https://docs.nvidia.com/deploy/nvml-api/group__nvmlDeviceQueries.html#group__nvmlDeviceQueries_1g90843d79066e66430ecb5929c698ce09
gpuInstalled=True
try:
  from pynvml import *
except NVMLError:
  gpuInstalled=False

running = True

## WirelessStatus.
#
#  Uses iwconfig to retrieve wireless signal strength and then publishes it.
class WirelessStatus:
    ##  Initializes wirelessSignalStatusMSG
    def __init__(self):
        self.linkQuality = 0
        self.maxQuality = 0
    ##  Retrieves wireless signal strength and stores it in the MSG format.
    def getValues(self):
        # outputString = commands.getoutput("sudo iwconfig | grep 'Link Quality='")
        outputString = subprocess.getoutput("sudo iwconfig | grep 'Link Quality='")
        linkQualityLocation = outputString.find("Link Quality=")
        if linkQualityLocation >= 0 and outputString[linkQualityLocation+13:linkQualityLocation+15].isdigit() and outputString[linkQualityLocation+16:linkQualityLocation+18].isdigit():
            self.linkQuality = int(outputString[linkQualityLocation+13:linkQualityLocation+15])
            self.maxQuality = int(outputString[linkQualityLocation+16:linkQualityLocation+18])

## PowerStatus.
#
#  Uses acpi to retrieve battery status and then publishes it.
class PowerStatus:
  ##  Initializes powerStatusMSG
  def __init__(self):
    self.percentage = 0
    self.valid = True
##  Retrieves battery status and stores it in the MSG format.
  def getValues(self):
    outputString = subprocess.getoutput("acpi")
    if outputString == "No support for device type: power_supply":
      self.valid = False
    else:
      self.valid = True
    percentageLocation = outputString.find("%")
    if percentageLocation >= 0:
#            print outputString[percentageLocation-2:pPubercentageLocation]
      self.percentage = int(outputString[percentageLocation-2:percentageLocation])

## M4ATXPowerStatus
#
#  Uses M4API to check compute box battery status and power supply diagnostics
class M4ATXPowerStatus:
    def __init__(self):
      self.percentage = 0
      self.VIN = 0
      self.V33 = 0
      self.V5  = 0
      self.V12 = 0
      self.temp = 0
      self.valid = True
    def getValues(self):
      if self.valid == False:
        return
      try:
        output = subprocess.check_output("m4ctl", shell=True)
      except (subprocess.CalledProcessError, e):
        self.valid = False
        if e.returncode == 127:
          print("m4ctl gave error code 127")
        else :
          print("Unknown error code from m4ctl: ", e.returncode)
      else:
        for line in output.split("\n"):
          tokens = line.split("\t")
          if len(tokens) == 2:
            if tokens[0] == "VIN:":
              self.valid = True
              self.VIN = float(tokens[1])
              # Battery max voltage 25, fully depleted 19, 6 volt span
              self.percentage = min( 100, ( ( float(tokens[1]) - 19 ) / 6.0 ) * 100)
            if tokens[0] == "33V:":
              self.valid = True
              self.V33 = float(tokens[1])
            if tokens[0] == "5V:":
              self.valid = True
              self.V5 = float(tokens[1])
            if tokens[0] == "12V:":
              self.valid = True
              self.V12 = float(tokens[1])
            if tokens[0] == "TEMP:":
              self.valid = True
              self.temp = float(tokens[1])

## TempStatus.
#
#  Uses sensors to retrieve CPU temperature and then publishes it.
class TempStatus:
  ##  Initializes tempStatusMSG
  def __init__(self):
      self.cpuTemp = 0
      self.fanSpeed = 0
  ##  Retrieves CPU temperature and fan speed and stores it in the MSG format.
  def getValues(self):
    outputString = subprocess.getoutput("sensors | grep \"Core 0:\"")
    cpuTempLocation = outputString.find("+")
    if cpuTempLocation >= 0:
      try:
        self.cpuTemp1 = float(outputString[cpuTempLocation+1:cpuTempLocation+5])
      except ValueError:
        self.cpuTemp1 = -1.0
    else:
      self.cpuTemp1 = 0.0
    outputString = subprocess.getoutput("sensors | grep \"Core 1:\"")
    cpuTempLocation = outputString.find("+")
    if cpuTempLocation >= 0:
      try:
          self.cpuTemp2 = float(outputString[cpuTempLocation+1:cpuTempLocation+5])
      except ValueError:
          self.cpuTemp2 = -1.0
    else:
      self.cpuTemp2 = 0.0
    outputString = subprocess.getoutput("sensors | grep \"Core 2:\"")
    cpuTempLocation = outputString.find("+")
    if cpuTempLocation >= 0:
      try:
          self.cpuTemp3 = float(outputString[cpuTempLocation+1:cpuTempLocation+5])
      except ValueError:
          self.cpuTemp3 = -1.0
    else:
      self.cpuTemp3 = 0.0
    outputString = subprocess.getoutput("sensors | grep \"Core 3:\"")
    cpuTempLocation = outputString.find("+")
    if cpuTempLocation >= 0:
      try:
        self.cpuTemp4 = float(outputString[cpuTempLocation+1:cpuTempLocation+5])
      except ValueError:
        self.cpuTemp4 = -1.0
    else:
      self.cpuTemp4 = 0.0
    self.cpuTemp = max(self.cpuTemp1, self.cpuTemp2, self.cpuTemp3, self.cpuTemp4)

    outputString = subprocess.getoutput("sensors | grep \"fan1:\"")
    split = outputString.split()
    if(len(split) >= 2):
      self.fanSpeed = split[1]
    else:
      self.fanSpeed = " "

    outputString = subprocess.getoutput("sensors | grep \"fan2:\"")
    split = outputString.split()
    if(len(split) >= 2):
      self.fanSpeed += "/" + split[1]
    else:
      self.fanSpeed += "/ "


    outputString = subprocess.getoutput("sensors | grep \"fan3:\"")
    split = outputString.split()
    if(len(split) >= 2):
      self.fanSpeed += "/" + split[1]
    else:
      self.fanSpeed += "/ "

class GpuStatus:
  def __init__(self):
    self.gpuInstalled = True      
    if gpuInstalled==True:
      try:
        nvmlInit()
      except NVMLError as e:
        self.gpuInstalled=False
    self.device = "N/A"
    self.driver = "N/A"
    self.memory = "N/A"
    self.fan = "N/A"
    self.temp = "N/A"
    self.utilization = "N/A"
    self.power = "N/A"
  def convert_size(self, size_bytes):
   if size_bytes == 0:
     return "0B"
   size_name = ("B", "KB", "MB", "GB", "TB", "PB", "EB", "ZB", "YB")
   i = int(math.floor(math.log(size_bytes, 1024)))
   p = math.pow(1024, i)
   s = round(size_bytes / p, 2)
   return "%s%s" % (s, size_name[i])

  def getValues(self):
      #only try to query GPU if nvml was sucessfully init
      if self.gpuInstalled==True:
        # assumes only 1 GPU installed, at index 0          
        handle = nvmlDeviceGetHandleByIndex(0)
        
        try:
          self.device = nvmlDeviceGetName(handle)
        except NVMLError as e:
          self.device = str(e)

        try:
          self.driver = nvmlSystemGetDriverVersion()
        except NVMLError as e:
          self.driver = str(e)

        try:
          memory = nvmlDeviceGetMemoryInfo(handle)
          self.memory = self.convert_size(memory.used)+"/"+self.convert_size(memory.free)+"/"+self.convert_size(memory.total)
        except NVMLError as e:
          self.memory = str(e)

        try:
          fanSpeed = nvmlDeviceGetFanSpeed(handle)
          self.fan = str(fanSpeed)+"%"
        except NVMLError as e:
          self.fan = str(e)

        try:
          temp = nvmlDeviceGetTemperature(handle, NVML_TEMPERATURE_GPU)
          self.temp  = str(temp)+"C"
        except NVMLError as e:
          self.temp = str(e)

        try:
          utilization = nvmlDeviceGetUtilizationRates(handle)
          self.utilization = str(utilization.gpu)+"%"
        except NVMLError as e:
          self.utilization = str(e)
        
        try:
          power = nvmlDeviceGetPowerUsage(handle)
          #convert reading in milliwatt to W
          self.power = str(power/1000.0)+"W"
        except NVMLError as e:
          self.power = str(e)



class ARStatus(Node):

        def __init__(self):
            super().__init__('SystemStatus')

            self.getParams()
            
            self.wirelessStatusPublisher = WirelessStatus()
            self.powerStatusPublisher = PowerStatus()
            self.tempStatusPublisher = TempStatus()
            self.powerSupplyHandler = M4ATXPowerStatus()

            self.powerSupplyHandler.valid=False
            self.gpuHandler = GpuStatus()

            self.hostname = socket.gethostname()

            self.pub = self.create_publisher(DiagnosticArray, 'diagnostics', 1)


            self.valOK = int(0).to_bytes(1, 'big')
            self.valWarn = int(1).to_bytes(1, 'big')
            self.valError = int(2).to_bytes(1, 'big')
            self.valStale = int(3).to_bytes(1, 'big')
            
            self.array = DiagnosticArray()
            self.status = DiagnosticStatus(name='systemStatus',\
                            level=self.valOK,\
                            message='',
                            hardware_id=self.hostname)

            self.array.status = [self.status]

            self.timer = self.create_timer(0.5, self.timer_callback)

        def getParams(self):
            self.declare_parameter('cpuTempHigh', 85.0)
            self.declare_parameter('cpuTempCrit', 95.0)
            self.declare_parameter('batteryLow', 50.0)
            self.declare_parameter('batteryCrit', 20.0)
            self.declare_parameter('wirelessLow', 10.0)
            self.declare_parameter('wirelessCrit', 0.0)
            self.declare_parameter('dataDrive', '/media/data')
            self.declare_parameter('dataDriveSpaceHighPercent', 75.0)
            self.declare_parameter('dataDriveSpaceCritPercent', 90.0)

            self.cpuTempHigh = self.get_parameter('cpuTempHigh').get_parameter_value().double_value
            self.cpuTempCrit = self.get_parameter('cpuTempCrit').get_parameter_value().double_value
            self.batteryLow = self.get_parameter('batteryLow').get_parameter_value().double_value
            self.batteryCrit = self.get_parameter('batteryCrit').get_parameter_value().double_value
            self.wirelessLow = self.get_parameter('wirelessLow').get_parameter_value().double_value
            self.wirelessCrit = self.get_parameter('wirelessCrit').get_parameter_value().double_value

            self.dataDrive = self.get_parameter('dataDrive').get_parameter_value().string_value
            self.dataDriveHighPercent = self.get_parameter('dataDriveSpaceHighPercent').get_parameter_value().double_value
            self.dataDriveSpaceCritPercent = self.get_parameter('dataDriveSpaceCritPercent').get_parameter_value().double_value

        def setLevel(self, level):
            if (self.status.level < level):
              self.status.level = level

        def timer_callback(self):
            self.status.values = []
            self.status.level = self.valOK

            self.wirelessStatusPublisher.getValues()
            self.powerStatusPublisher.getValues()
            self.tempStatusPublisher.getValues()
            self.powerSupplyHandler.getValues()

            if self.powerStatusPublisher.valid:
                self.status.values.append(KeyValue(key='Laptop Battery Level', value=str(self.powerStatusPublisher.percentage)))
                if powerStatusPublisher.percentage <= self.batteryCrit:
                    self.setLevel(self.valError)
                    self.status.values.append(KeyValue(key='BATTERY CRITICALLY LOW', value=chr(2)))
                elif powerStatusPublisher.percentage <= self.batteryLow:
                    self.setLevel(self.valWarn)
                    self.status.values.append(KeyValue(key='BATTERY LOW', value=chr(1)))

            if self.powerSupplyHandler.valid:
                self.status.values.append(KeyValue(key='Power Supply Battery Level', value=str(self.powerSupplyHandler.percentage)))
                if self.powerSupplyHandler.percentage <= self.batteryCrit:
                    self.status.values.append(KeyValue(key='Power Supply Input CRITICALLY LOW', value=chr(2)))
                    self.setLevel(self.valError)
                elif self.powerSupplyHandler.percentage <= self.batteryLow:
                    self.status.values.append(KeyValue(key='Power Supply Input LOW', value=chr(1)))
                    self.setLevel(self.valWarn)
                self.status.values.append(KeyValue(key='Power Supply VIN', value=str(self.powerSupplyHandler.VIN)))
                self.status.values.append(KeyValue(key='Power Supply 3.3v Rail', value=str(self.powerSupplyHandler.V33)))
                self.status.values.append(KeyValue(key='Power Supply 5v Rail', value=str(self.powerSupplyHandler.V5)))
                self.status.values.append(KeyValue(key='Power Supply 12v Rail', value=str(self.powerSupplyHandler.V12)))
                self.status.values.append(KeyValue(key='Power Supply Temp', value=str(self.powerSupplyHandler.temp)))
                    
            self.status.values.append(KeyValue(key='CPU Temp', value=str(self.tempStatusPublisher.cpuTemp)))
            if self.tempStatusPublisher.cpuTemp >= self.cpuTempCrit:
                self.setLevel(self.valError)
                self.status.values.append(KeyValue(key='CPU CRITICALLY HOT', value=chr(2)))
            elif self.tempStatusPublisher.cpuTemp >= self.cpuTempHigh:
                self.status.values.append(KeyValue(key='CPU HOT', value=chr(1)))
                self.setLevel(self.valWarn)
            self.status.values.append(KeyValue(key='Fan 1/2/3 (case/CPU/case) Speeds', value=self.tempStatusPublisher.fanSpeed))

            # WiFi status is not currently published by our driver
            self.status.values.append(KeyValue(key='WiFi Quality', value=str(self.wirelessStatusPublisher.linkQuality)))
            self.status.values.append(KeyValue(key='WiFi Max Quality', value=str(self.wirelessStatusPublisher.maxQuality)))
            if self.wirelessStatusPublisher.linkQuality <= self.wirelessCrit:
                self.setLevel(self.valError)
                self.status.values.append(KeyValue(key='WiFi CRITICALLY WEAK', value=chr(2)))
            elif self.wirelessStatusPublisher.linkQuality <= self.wirelessLow:
                self.setLevel(self.valWarn)
                self.status.values.append(KeyValue(key='WiFi WEAK', value=chr(1)))

            #get available dick space in /media/data
            try:                
                diskUsage = subprocess.check_output("df -h " + self.dataDrive, shell=True)              
                lines = diskUsage.split(b'\n')                
                #remove extra spaces and split on remaning/media/data spaces
                line0 = (" ".join(str(lines[0]).split())).split(" ")
                line1 = (" ".join(str(lines[1]).split())).split(" ")
                for a, b in zip(line0, line1):
                  #check disc usage %              
                  if a == "Use%":
                      discPercent = int(b.strip("%"))
                      if discPercent > self.dataDriveHighPercent:
                          self.setLevel(self.valWarn)
                          self.status.values.append(KeyValue(key=self.dataDrive + " filling up!", value=chr(1)))              
                      if discPercent > self.dataDriveSpaceCritPercent:
                          self.setLevel(self.valError)
                          self.status.values.append(KeyValue(key=self.dataDrive + " almost full!", value=chr(2)))  

                #only add part of the df -h output to diagnostics              
                if a not in ["Filesystem", "Mounted", "on"]:
                    self.status.values.append(KeyValue(key=a + " " + self.dataDrive, value=b))
            except (subprocess.CalledProcessError, e):
                self.status.values.append(KeyValue(key="Error reading disc usage", value=e))
            if gpuInstalled==True:
                self.gpuHandler.getValues()
                if self.gpuHandler.gpuInstalled==True:                    
                    self.status.values.append(KeyValue(key="GPU", value=self.gpuHandler.device.decode()))
                    self.status.values.append(KeyValue(key="GPU driver", value=self.gpuHandler.driver.decode()))
                    self.status.values.append(KeyValue(key="GPU memory used/free/total", value=self.gpuHandler.memory))
                    self.status.values.append(KeyValue(key="GPU fan", value=self.gpuHandler.fan))
                    self.status.values.append(KeyValue(key="GPU temp", value=self.gpuHandler.temp))
                    self.status.values.append(KeyValue(key="GPU utilization", value=self.gpuHandler.utilization))
                    self.status.values.append(KeyValue(key="GPU power draw", value=self.gpuHandler.power))
                else:
                    self.status.values.append(KeyValue(key="Nvidia Management library init failed", value="GPU not installed?"))
            else:
                self.status.values.append(KeyValue(key="Nvidia Management library not imported", value="GPU not installed?"))
                
            self.pub.publish(self.array)


    
def main(args=None):
  rclpy.init()
  node = ARStatus()
  rclpy.spin(node)

  print("SystemStatus: Shutting down")

if __name__ == '__main__':
    main()



