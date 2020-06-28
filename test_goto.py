import sys
from MotorizedPlatform import MotorizedPlatform
from Navigation import Navigation
from PositionWatcher import PositionWatcher
import Adafruit_PCA9685
from time import sleep

pwm = Adafruit_PCA9685.PCA9685()
platform = MotorizedPlatform(pwm)
positionWatcher = PositionWatcher()

def app():  
  platform.stop()
  sleep(0.2)
  
  positionWatcher.start()
  print('started position watcher')
  
  nav = Navigation(platform, positionWatcher)

  nav.goTo(-400, 1000)

try:
  app()
except KeyboardInterrupt:
  print("KeyboardInterrupt")
  positionWatcher.stop()
  platform.stop()
  sys.exit()
  
