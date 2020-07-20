from math import *
from time import sleep

class Navigation:
  def __init__(self, platform, positionWatcher):
    self.platform = platform
    self.positionWatcher = positionWatcher
    self.enabled = False

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

  def goTo(self, targetX, targetY, speed = 30, threshold = 5):
    if not self.positionWatcher.isEnabled():
      positionWatcher.start()

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
      x, y, theta = self.positionWatcher.computePosition()
      dist = sqrt((targetX - self.x)**2 + (targetY - self.y)**2)
      print(round(self.x, 0), round(self.y, 0), round(dist, 0))

      if initialDist == None:
        initialDist = dist
      if dist <= threshold:
        self.done = True
      else:
        targetAngle = atan2(targetY - self.y, targetX - self.x)
        #print(str(degrees(targetAngle)) + " new computed angle")
        s = self.getPlatformSpeed(initialDist, dist, speed, minSpeed)
        #print("speed", s)
        b = self.getSpeedFromAngle(targetAngle, s)
        #print(b)
        self.setSpeed(b)

  def goToPath(self, path, speed = 80, threshold = 5):
    for node in path:
      if len(node) > 2:
        speed = node[2]
      if len(node) > 3:
        threshold = node[3]
      self.goTo(node[0], node[1], speed, threshold)
      sleep(0.8)

    self.stop()
    print('Done')
  