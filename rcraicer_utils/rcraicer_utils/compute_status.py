#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

import time
import subprocess
import string
from rcraicer_msgs.msg import ComputeStatus

class MachineStats():
    def __init__(self):
        self.hostName = ""
        self.memTotal = ""
        self.memFree = ""        
        self.memPct = 0.
        self.statTime = ""
        self.cpu = ["", "", ""]
        self.ssid = ""
        self.bitRate = ""
        self.linkQuality = ""
        self.powerLevel = ""

    def __repr__(self):
        return self.memTotal + " : " + self.memFree + " : " + str(self.cpu) + " : " + self.statTime

    def parseStats(self, stats):
        lines = stats.split('\n')
        self.parseCPUStats(lines)
        self.parseMemStats(lines)
        self.parseTime(lines)


    def parseCPUStats(self, stats):
        for line in stats:
            if "load average:" in line:
                ind = line.find("load average:")                
                line = line[ind+13:]                
                cpuInfo = line.split(",")                

                sz = len(cpuInfo)

                if sz > len(self.cpu):
                    sz = len(self.cpu)                
                
                for i in range(sz):                    
                    cpuStr = self.escape_ansi(cpuInfo[i]).strip()                                        
                    self.cpu[i] = str(cpuStr)

                return    

    def escape_ansi(self, info):        
        r = ""
        dataStarted = False

        for c in info:            
            if ord(c) == 32:
                dataStarted = True
            elif ord(c) < 28:
                dataStarted = False
            
            if dataStarted:
                r += c            
        return r        

    
    def parseMemStats(self, stats):                
        for line in stats:
            if "KiB Mem" in line:
                sInd = line.find("KiB Mem :")
                eInd = line.find("total")
                
                totMem = line[sInd+9:eInd]                
                self.memTotal = self.escape_ansi(totMem).strip()
                                
                sInd = line.find("total,")
                eInd = line.find("free")

                freeMem = line[sInd+6:eInd]                
                self.memFree = self.escape_ansi(freeMem).strip()

                try:
                    totMem = float(self.memTotal)
                    totFree = float(self.memFree)
                    self.memPct = int((totMem - totFree) / totMem * 100)
                except:
                    self.memPct = -1

                return

    def parseWifiStats(self, stats):
        lines = stats.split('\n')
        for line in lines:            
            if "ESSID:" in line:
                sInd = line.find("ESSID:")
                eInd = line.find('"',  sInd+7)
                self.ssid = line[sInd+7: eInd].strip()

            elif "Bit Rate=" in line:
                sInd = line.find("Bit Rate=")
                eInd = line.find('Tx',  sInd+9)
                self.bitRate = line[sInd+9: eInd].strip()

            elif "Link Quality" in line:
                sInd = line.find("Link Quality=")
                eInd = line.find("Signal",  sInd+13)
                self.linkQuality = line[sInd+13: eInd].strip()


    
    def parseTime(self, stats):
        for line in stats:
            if "top -" in line:
                sInd = line.find("top -")
                eInd = line.find("up")
                line = line[sInd + 6:eInd]
                self.statTime = line.strip()
                return

    def parseHostName(self, stats):
        self.hostName = stats

    def parsePowerStats(self, stats):
        sInd = stats.find(":")
        eInd = stats.find('\n')

        mode = stats[sInd+1:eInd].strip()
        level = stats[eInd+1:].strip()
        try:
            self.powerLevel = mode + " (" + level +")"
        except:
            self.powerLevel = ""

    def getMessage(self):
        msg = ComputeStatus()
        msg.host_name = self.hostName
        msg.link_quality = self.linkQuality
        msg.ssid = self.ssid
        msg.bit_rate = self.bitRate
        msg.mem_total = self.memTotal
        msg.mem_free = self.memFree
        msg.mem_pct = str(self.memPct)
        msg.stat_time = self.statTime
        msg.cpu1 = self.cpu[0]
        msg.cpu2 = self.cpu[1]
        msg.cpu3 = self.cpu[2]
        msg.power_level = self.powerLevel

        return msg

class ComputerStatus(Node):
    def __init__(self):
        super().__init__("rcraicer_comp_status")        
        self.machineStats = MachineStats()        
        self.pub = self.create_publisher(ComputeStatus, 'comp_status', 10)        

        self.declare_parameter("freq", 1.)
        self.declare_parameter("comp_name", "sct")
        self.declare_parameter("power_mgmt", True)

        self.freq = self.get_parameter("freq").get_parameter_value().double_value
        self.compName = self.get_parameter("comp_name").get_parameter_value().string_value
        self.powerMgmt = self.get_parameter("power_mgmt").get_parameter_value().bool_value

        self.timer = self.create_timer(self.freq, self.publishStatus)

    def runCommand(self, command):
        retval = -1

        try:
            retval = subprocess.check_output(command)                        
        except Exception as exc:
            retval = None            
            self.get_logger().warn("exception during command: " + str(exc))


        return retval

    def getHostName(self):
        command = "hostname"
        retval = self.runCommand(command)

        if retval is not None:
            self.machineStats.parseHostName(str(retval))

    def getPowerSetting(self):
        command = ["/usr/sbin/nvpmodel","-q"]

        retval = self.runCommand(command)

        if retval is not None:
            self.machineStats.parsePowerStats(str(retval))

    def getWifiStats(self):
        # command = "sudo iwconfig"
        command = "iwconfig"

        retval = self.runCommand(command)

        if retval is not None:
            self.machineStats.parseWifiStats(str(retval))

    def getCompStats(self):        
        command = ["top", "-n 1", "-b"]

        retval = self.runCommand(command)

        if retval is not None:
            self.machineStats.parseStats(str(retval))        
        else:
            self.get_logger().warn("Comp stats failed")            

    def publishStatus(self):        
        self.getHostName()
        self.getWifiStats()
        self.getCompStats()

        if self.powerMgmt:
            self.getPowerSetting()
        
        self.pub.publish(self.machineStats.getMessage())

if __name__ == '__main__':
    rclpy.init()

    comp = ComputerStatus()    
    rclpy.spin(comp)

    comp.destroy_node()
    rclpy.shutdown
    






