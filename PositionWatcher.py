from gpiozero import DigitalInputDevice
from threading import Thread
from time import sleep
from math import *

from time import sleep
from BMI160_i2c import Driver


class PositionWatcher:
    perimeter = 60*pi
    theta = pi / 2
    #theta = (0, 0)
    x = 0
    y = 0
    
    # left
    phaseA = DigitalInputDevice(6, True)
    phaseB = DigitalInputDevice(16, True)
    
    # right
    phaseC = DigitalInputDevice(20, True)
    phaseD = DigitalInputDevice(21, True)
    
    leftTicks = 0
    rightTicks = 0
    
    leftState = (0, 0)
    leftOldState = (0, 0)
    
    rightState = (0, 0)
    rightOldState = (0, 0)
    
    watchPositionThread = None
    watchTicksThread = None
    enabled = True
    
    oldTicks = (0, 0)
    
    onPositionChangedHandler = None
    
    isTurning = False

    
    L = 120
    l = 161
    
    def watchTicks(self):
        while self.enabled:
            leftFetchedState = (self.phaseA.value, self.phaseB.value)
            rightFetchedState = (self.phaseC.value, self.phaseD.value)
            if leftFetchedState != self.leftState:
                self.leftState = leftFetchedState
                
                if self.leftState[0] == self.leftOldState[1]:
                    self.leftTicks -= 1
                else:
                    self.leftTicks += 1

                self.leftOldState = self.leftState

            if rightFetchedState != self.rightState:
                self.rightState = rightFetchedState

                if self.rightState[0] == self.rightOldState[1]:
                    self.rightTicks -= 1
                else:
                    self.rightTicks += 1

                self.rightOldState = self.rightState
            

    def watchPosition(self):
        while self.enabled:
            # LEFT = SIDE
            # RIGHT = BACK
            newTicks = (self.leftTicks, self.rightTicks)
            if (newTicks != self.oldTicks):
                deltaTicks = (newTicks[0] - self.oldTicks[0],
                              newTicks[1] - self.oldTicks[1])
                self.oldTicks = newTicks
                leftDistance = deltaTicks[0] / 2400 * self.perimeter
                rightDistance = deltaTicks[1] / 2400 * self.perimeter
                # t1 = (leftDistance + rightDistance) / 2
                # alpha = (rightDistance - leftDistance) / self.axialDistance
                
                self.x += sin(self.theta)*-rightDistance + cos(self.theta)*leftDistance
                self.y += cos(self.theta)*rightDistance + sin(self.theta)*leftDistance
                
                if self.onPositionChangedHandler != None:
                    self.onPositionChangedHandler(self.x, self.y, self.theta)
            sleep(0.01)


    def watchOrientation(self):
        sensor = Driver()
        sensor.autoCalibrateGyroOffset()
        print("Gyro - Calibration done!")
        timeInterval = 0.05
        self.theta = pi/2
        while (self.enabled):
            sleep(timeInterval)
            self.theta += radians(((sensor.getRotationZ()[0] * 250.0) / 32768.0) * timeInterval)


    def start(self):
        self.enabled = True
        self.watchTicksThread = Thread(target=self.watchTicks)
        self.watchTicksThread.start()
        self.watchPositionThread = Thread(target=self.watchPosition)
        self.watchPositionThread.start()
        self.watchOrientationThread = Thread(target=self.watchOrientation)
        self.watchOrientationThread.start()
        
    def stop(self):
        self.enabled = False

    def getPos(self):
        return (self.x, self.y)

    def getPosRot(self):
        return (self.x, self.y, self.theta * 180/pi)

    def changeIsTurning(self, p):
        self.isTurning = p
        return self.isTurning

    def getOrientation(self):
        return (self.theta)

    def getOrientationDeg(self):
        return (self.theta * 180/pi)
      
    def setOnPositionChangedHandler(self, handler):
        self.onPositionChangedHandler = handler

    def getTicks(self):
        return [self.leftTicks, self.rightTicks]
 
