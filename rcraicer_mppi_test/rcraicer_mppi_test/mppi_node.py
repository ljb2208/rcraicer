from numpy.core.fromnumeric import std
import rclpy
from rclpy.node import Node
import numpy as np
import torch
import math

from dyn_model import DynModel
from rcraicer_msgs.msg import SimState


class MPPINode(Node):
    def __init__(self):
        super().__init__('mppi_python_node')
        self.simStateSub = self.create_subscription(SimState, 'sim_state', self.onSimState, 10)
        self.model = DynModel()
        self.model.load_state_dict(torch.load("/home/lbarnett/ros2_ws/src/rcraicer/rcraicer_mppi/models/dynmodel_states2.pth"))
        self.model.eval()

        self.loadCostMap()

        self.rollouts = 20
        self.particles = 100

        self.throttleDev = 0.3
        self.steerDev = 0.3
        self.timeStep = 0.05

        self.desiredSpeed = 20.0
        self.crashCoefficient = 10000.0
        self.throttleCoefficient = 0.0
        self.steeringCoefficient = 0.0

        self.trackCoefficient = 200.0

        self.maxSlipAngle = 2.0
        self.slipPenalty = 10.0
        self.runFilter()


    def runFilter(self):
        throttle = 0.0
        steer = 0.0        
        # throttleSample = torch.empty(self.particles).normal_(mean=throttle, std=self.throttleDev)
        # steerSample = torch.empty(self.particles).normal_(mean=steer, std=self.steerDev)

        crash = torch.zeros(self.particles)
        costs = torch.zeros(self.particles, self.rollouts)
        controls = torch.zeros(self.particles, self.rollouts, 2)


        stateData = torch.zeros(self.particles, self.rollouts, 9)

        throttleRand = torch.empty(self.particles, self.rollouts).normal_(mean=0.0, std=self.throttleDev).clamp(-1.0, 1.0)
        steeringRand = torch.empty(self.particles, self.rollouts).normal_(mean=0.0, std=self.steerDev).clamp(-1.0, 1.0)        



        for j in range(self.rollouts):
            for i in range(self.particles):
                # print(inputData[i, :])
                # print(output[i, :])

                stateData[:,j, 0] = stateData[:,j, 0] + steeringRand[:, j]
                stateData[:,j, 1] = stateData[:,j, 1] + throttleRand[:, j]

                stateData[:,j, 0].clamp(-1.0, 1.0)
                stateData[:,j, 1].clamp(-1.0, 1.0)
                
                # inputData = torch.zeros(self.particles, 9)        
                # inputData[:, 0] = torch.empty(self.particles).normal_(mean=steer, std=self.steerDev).clamp(-1.0, 1.0)
                # inputData[:, 1] = torch.empty(self.particles).normal_(mean=throttle, std=self.throttleDev).clamp(-1, 1.0)

                output = self.model.forward(stateData[:, j, :])

                pfSteer = stateData[i, j, 0].item()
                pfThrottle = stateData[i, j, 1].item()
                pfXPos = stateData[i, j, 2].item()
                # pfYPos = inputData[i, 3].item()
                pfYPos = -2.0
                pfYaw = stateData[i, j, 4].item()
                pfRoll = stateData[i, j, 5].item()

                pfXVel = output[i, 0].item()
                pfYVel = output[i, 1].item()
                pfYawRate = output[i, 2].item()

                cx, cy, cyaw = self.calculateStateDeriv(pfXVel, pfYVel, pfYaw, pfYawRate)

                x, y = self.convertCoords(cx, cy)
                cmVal = self.getCostMapValue(x, y)

                cost = self.getSpeedCost(pfXVel, pfYVel)
                cost += self.getControlCost(throttle, steer, pfThrottle, pfSteer)
                cost += self.getTrackCost(cmVal)
                cost += self.getCrashCost(cmVal)
                cost += self.getStabilizingCost(pfXVel, pfYVel)
                cost += self.getControlCost(throttle, steer, pfThrottle, pfSteer)

                if (cost > self.crashCoefficient):
                    crash[i] = 1.0

                if (j < self.rollouts):
                    stateData[i, j+1, 0]= pfSteer
                    stateData[i, j+1, 1] = pfThrottle
                    stateData[i, j+1, 2] = cx
                    stateData[i, j+1, 3] = cy
                    stateData[i, j+1, 4] = cyaw
                    stateData[i, j+1, 5] = 
                    stateData[i, j+1, 6] = pfXVel
                    stateData[i, j+1, 7] = pfYVel
                    stateData[i, j+1, 8] = pfYawRate
                
                costs[i, j] = costs[i, j] + cost

            print(str.format(" particle {}: cost {}", i, cost))

    def calculateStateDeriv(self, xVel, yVel, yaw, yawDer):
        x = math.cos(yaw) * xVel - math.sin(yaw) * yVel * self.timeStep
        y = math.sin(yaw) * xVel + math.cos(yaw) * yVel * self.timeStep
        newYaw = -yawDer * self.timeStep

        return x, y, newYaw    

    
    def getNewPos(self, xPos, yPos, xVel, yVel):
        xNew = xPos + xVel * self.timeStep
        yNew = yPos + yVel * self.timeStep

        return xNew, yNew

    def getControlCost(self, throttle, steer, newThrottle, newSteer):
        costs = self.throttleCoefficient * newThrottle * (throttle - newThrottle) / (math.pow(self.throttleDev, 2))
        costs += self.steeringCoefficient * newSteer * (steer - newSteer) / (math.pow(self.steerDev, 2))
        
        return costs

    
    def getSpeedCost(self, xVel, yVel):
        speed = math.sqrt(math.pow(xVel, 2) + math.pow(yVel, 2))
        err = speed - self.desiredSpeed
        return math.pow(err, 2)

    def getTrackCost(self, cmVal):
        return cmVal * self.trackCoefficient

    def getCrashCost(self, cmVal):
        if (cmVal >= 1.0):
            return self.crashCoefficient
        
        return 0.0

    def getStabilizingCost(self, xVel, yVel):
        cost = 0.0

        slip = -math.atan(yVel/abs(xVel)) 

        cost = self.slipPenalty*math.pow(slip, 2)

        if (slip > self.maxSlipAngle):
            cost += self.crashCoefficient

        return cost    


    def onSimState(self, msg):
        print(msg.throttle)

    def getCostMapValue(self, xPos, yPos):
        index = yPos * self.cmStep + xPos
        return self.cmData[index]

    def convertCoords(self, xPos, yPos):
        x = int(round(xPos * self.pixelsPerMeter - self.xBounds[0]))
        y = int(round(yPos * self.pixelsPerMeter - self.yBounds[0]))

        return x, y


    def loadCostMap(self):
        data = np.load("/home/lbarnett/ros2_ws/src/rcraicer/costmap.npz")

        self.pixelsPerMeter = data["pixelsPerMeter"][0]
        self.xBounds = data["xBounds"]
        self.yBounds = data["yBounds"]
        self.cmData = data["channel0"]        
        self.cmStep = int(self.xBounds[1] - self.xBounds[0])



def main(args=None):
    rclpy.init(args=args)
    mppi_node = MPPINode()
    rclpy.spin(mppi_node)

    mppi_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()