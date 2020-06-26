from math import atan2, hypot, pi, cos, degrees
from time import sleep

class Navigation:
  def __init__(self, platform, positionWatcher):
    self.platform = platform
    self.positionWatcher = positionWatcher
    self.enabled = False
    
  def getSpeedFromAngle(self, targetAngle, speed):
    # marks = [
    #   0,
    #   pi/2,
    #   pi,
    #   -pi/2
    # ]
    # cmds = [
    #   [ 1,  1,  1,  1], # east
    #   [ 1, -1, -1,  1], # north
    #   [-1, -1, -1, -1], # west
    #   [-1,  1,  1, -1]  # south
    # ]
    # motorsSpeed = []
    # for n in range(4):
    #   s = 0
    #   for i in range(4):
    #     coef = cos(abs(targetAngle - marks[i]))
    #     s += cmds[i][n] * coef * speed
    #   s /= 4
    #   s = round(s, 3)
    #   if s == 0: s = 0
    #   motorsSpeed.append(s)
    
    def generateCoeffs(a): return cos(abs(targetAngle - a))
  
    eastCoef = generateCoeffs(0)-pi/2
    northCoef = generateCoeffs(pi/2)-pi/2
    westCoef = generateCoeffs(pi)-pi/2
    southCoef = generateCoeffs(-pi/2)-pi/2
    cmds = [
      [ eastCoef,  eastCoef,   eastCoef,   eastCoef],
      [ northCoef, -northCoef, -northCoef, northCoef],
      [-westCoef,  -westCoef,  -westCoef,  -westCoef],
      [-southCoef, southCoef,  southCoef,  -southCoef],
    ]
  
    motorsSpeed = []
    for n in range(4):
      sum = 0
      for i in range(4):
        sum += cmds[i][n] * speed
      motorsSpeed.append(round(sum / 4 * 2, 3))
    
    speedList = {
      'frontLeft': motorsSpeed[0],
      'frontRight': motorsSpeed[1],
      'backLeft': motorsSpeed[2],
      'backRight': motorsSpeed[3]
    }
    return speedList
    
  def goTo(self, targetX, targetY, speed = 30, threshold = 50):
    targetAngle = atan2(targetX, targetY)
    self.done = False
    print("> Navigation: going to (x: %(x)f y: %(y)f) with a angle of %(a)f deg" % {
      'x': targetX,
      'y': targetY,
      'a': degrees(targetAngle)
    })
    
    def onPositionChanged(x, y, theta):
      if self.done:
        return
      dist = hypot(targetX - x, targetY - y)
      print(x, y)
      print(dist)
      if dist <= threshold:
        self.done = True
        
    self.positionWatcher.setOnPositionChangedHandler(onPositionChanged)
    
    motorSpeed = self.getSpeedFromAngle(targetAngle, speed)
    print(motorSpeed)
    
    self.platform.setSpeed(motorSpeed)
    while not self.done:
      pass
    
    self.platform.stop()
    print('Done')