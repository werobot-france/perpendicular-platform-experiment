import sys
from gpiozero import DigitalInputDevice
from threading import Thread
from time import sleep
from math import *

import Adafruit_PCA9685
from time import sleep

class Main:
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
  
  watchPositionThread = None
  watchTicksThread = None
  enabled = True
  
  onPositionChangedHandler = None
  
  isTurning = False

  L = 195
  l = 215
  
  leftTicks = 0
  rightTicks = 0
  
  leftState = (0, 0)
  leftOldState = (0, 0)
  
  rightState = (0, 0)
  rightOldState = (0, 0)
  
  oldTicks = (0, 0)
  
  escSlots = [15, 12, 14, 13]
  pwmInterface = None
  
  done = False
  
  def __init__(self):
    self.pwmInterface = Adafruit_PCA9685.PCA9685()
    self.pwmInterface.set_pwm_freq(50)
    self.setSpeed([0, 0, 0, 0])
    sleep(0.5)
  
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

  def start(self):
    self.enabled = True
    self.watchTicksThread = Thread(target=self.watchTicks)
    self.watchTicksThread.start()
    
  # équivalent de la fonction map() de arduino
  def mappyt(self, x, inMin, inMax, outMin, outMax):
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin
  
  # fonction esc pour une vitesse de moteur de -100 à 100()
  def convertSpeedToEsc(self, speed):
    return round(self.mappyt(speed, 0, 100, 307, 410))
    
  def setSpeed(self, values):
    for i in range(len(values)):
      self.pwmInterface.set_pwm(
        self.escSlots[i],
        0,
        self.convertSpeedToEsc(values[i])
      )

  def getSpeedFromAngle(self, targetAngle, speed):
    marks = [0, pi/2, pi, -pi/2]
    cmds = [
      [ 1,  1,  1,  1], # east
      [ 1, -1, -1,  1], # north
      [-1, -1, -1, -1], # west
      [-1,  1,  1, -1]  # south
    ]
    motorsSpeed = []
    for n in range(4):
      s = 0
      for i in range(4):
        coef = cos(abs(targetAngle - marks[i]))
        s += cmds[i][n] * coef * speed
      s /= 4
      s = round(s, 3)
      if s == 0: s = 0
      motorsSpeed.append(s)
    return motorsSpeed

  def goTo(self, targetX, targetY, speed = 30, threshold = 5):
    minSpeed = 25
    if speed < minSpeed:
      speed = minSpeed
    self.done = False
    targetAngle = atan2(targetY, targetX)
    print("> Navigation: going to (x: %(x)f y: %(y)f) with a angle of %(a)f deg" % {
      'x': targetX,
      'y': targetY,
      'a': degrees(targetAngle)
    })
    #self.setSpeed(self.getSpeedFromAngle(targetAngle, speed))
    initialDist = None
    while not self.done:
      newTicks = (self.leftTicks, self.rightTicks)
      deltaTicks = (newTicks[0] - self.oldTicks[0], newTicks[1] - self.oldTicks[1])
      self.oldTicks = newTicks
      leftDistance = deltaTicks[0] / 2400 * self.perimeter
      rightDistance = deltaTicks[1] / 2400 * self.perimeter
      # t1 = (leftDistance + rightDistance) / 2
      # alpha = (rightDistance - leftDistance) / self.axialDistance
      
      self.x += sin(self.theta)*-rightDistance + cos(self.theta)*leftDistance
      self.y += cos(self.theta)*rightDistance + sin(self.theta)*leftDistance
      #dist = hypot(targetX - self.positionWatcher.x, targetY - self.positionWatcher.y)
      dist = sqrt((targetX - self.x)**2 + (targetY - self.y)**2)
      print(round(self.x, 0), round(self.y, 0), round(dist, 0))
      if initialDist == None:
        initialDist = dist
      if dist <= threshold:
        self.done = True
      else:
        #print("stringe")
        targetAngle = atan2(targetY - self.y, targetX - self.x)
        #print(str(degrees(targetAngle)) + " new computed angle")
        s = self.getPlatformSpeed(initialDist, dist, speed, minSpeed)
        print("speed", s)
        b = self.getSpeedFromAngle(targetAngle, s)
        #print(b)
        self.setSpeed(b)
    
    self.stop()
    print('Done')
  
  def stop(self):
    self.done = True
    #self.enabled = False
    self.setSpeed([0, 0, 0, 0])
    
  def stopWatch(self):
    self.enabled = False
    
  def getPlatformSpeed(self, initialDist, dist, maxSpeed, minSpeed):
    p = abs(initialDist - dist)
    if p <= 25:
      return self.saturation(0, 25, minSpeed, maxSpeed, p)
    else:
      l = maxSpeed - minSpeed
      k = 0.04
      o = 100
      return (l/(1+exp(-(k*(dist - o))))) + minSpeed
      
  
  def saturation(self, minX, maxX, minY, maxY, value):
    # minX = 10*10
    # maxX = 100*10
    # minY = 10
    # maxY = 100
    minX *= 10
    maxX *= 10
    if value <= minX:
      print('Very start thing case')
      return minY
    elif value >= maxX:
      print('Normal cruise')
      return maxY
    else:
      print('Start thing case')
      a = (maxY-minY)/(maxX - minX)
      b = minY - a*minX
      return a * value + b

  def goToPath(self, path, speed = 80, threshold = 5):
    for node in path:
      if len(node) > 2:
        speed = node[2]
      if len(node) > 3:
        threshold = node[3]
      self.goTo(node[0], node[1], speed, threshold)
      sleep(0.8)

main = Main()

def app():
  main.start()
  
  
  #main.goTo(450, 1100, 70)
  #main.goTo(-300, 600, 70)
  main.goToPath([
    [0, 300],
    [-300, 300],
    [-300, 600],
    [0, 600],
    [-200, 1100],
    [-600, 600],
    [0, 0],
  ])
  

try:
  app()
except KeyboardInterrupt:
  print("KeyboardInterrupt")
  main.stop()
  main.stopWatch()
  sys.exit()
  