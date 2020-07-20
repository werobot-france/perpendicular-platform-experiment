from gpiozero import DigitalInputDevice
from threading import Thread
from time import sleep
from math import *

from time import sleep
from BMI160_i2c import Driver

class PositionWatcher:
  backPerimeter = 90*pi
  lateralPerimeter = 60*pi
  theta = pi / 2
  #theta = (0, 0)
  x = 0
  y = 0
  
  # left (scotch bleu)
  phaseA = DigitalInputDevice(20, True)
  phaseB = DigitalInputDevice(21, True)
  
  # right
  phaseC = DigitalInputDevice(6, True)
  phaseD = DigitalInputDevice(16, True)

  # back  (scotch vert)
  phaseE = DigitalInputDevice(5, True)
  phaseF = DigitalInputDevice(19, True)
  
  leftTicks = 0
  rightTicks = 0
  backTicks = 0
  
  leftState = (0, 0)
  leftOldState = (0, 0)
  
  rightState = (0, 0)
  rightOldState = (0, 0)
  
  backState = (0, 0)
  backOldState = (0, 0)
  
  watchPositionThread = None
  watchTicksThread = None
  enabled = True
  
  oldTicks = (0, 0, 0)
  
  onPositionChangedHandler = None
  
  isTurning = False

  L = 140
  l = 215

  def __init__(self):
    print("Pos watcher init")
  
  def watchTicks(self):
    while self.enabled:
      leftFetchedState = (self.phaseA.value, self.phaseB.value)
      rightFetchedState = (self.phaseC.value, self.phaseD.value)
      backFetchedState = (self.phaseE.value, self.phaseF.value)
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

      if backFetchedState != self.backState:
        self.backState = backFetchedState

        if self.backState[0] == self.backOldState[1]:
          self.backTicks -= 1
        else:
          self.backTicks += 1

        self.backOldState = self.backState

  def watchPosition(self):
    while self.enabled:
      newTicks = (self.leftTicks, self.rightTicks, self.backTicks)
      if (newTicks != self.oldTicks):
        deltaTicks = (newTicks[0] - self.oldTicks[0],
                      newTicks[1] - self.oldTicks[1],
                      newTicks[2] - self.oldTicks[2],
                      )
        self.oldTicks = newTicks
        leftDistance = deltaTicks[0] / 2400 * self.lateralPerimeter
        rightDistance = deltaTicks[1] / 2400 * self.lateralPerimeter
        backDistance = deltaTicks[2] / 2400 * self.backPerimeter
        
        #self.x += sin(self.theta)*-rightDistance + cos(self.theta)*leftDistance
        #self.y += cos(self.theta)*rightDistance + sin(self.theta)*leftDistance
        t1 = (leftDistance + rightDistance) / 2
        alpha = (rightDistance - leftDistance) / self.l * 2
        self.theta = self.theta + alpha
        self.x = self.x + t1 * cos(self.theta)  #+ (backDistance - self.L*alpha) * cos(self.theta)
        self.y = self.y + t1 * sin(self.theta)  #+ (backDistance - self.L*alpha) * sin(self.theta)

        if self.onPositionChangedHandler != None:
            self.onPositionChangedHandler(self.x, self.y, self.theta)


  def start(self):
    self.enabled = True
    self.watchTicksThread = Thread(target=self.watchTicks)
    self.watchTicksThread.start()
    self.watchPositionThread = Thread(target=self.watchPosition)
    self.watchPositionThread.start()
      
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
