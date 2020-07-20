import sys
from MotorizedPlatform import MotorizedPlatform
from Navigation import Navigation
import Adafruit_PCA9685
from time import sleep

pwm = Adafruit_PCA9685.PCA9685()
platform = MotorizedPlatform(pwm)

def app():  
  platform.stop()
  sleep(0.2)
  platform.setSpeed([
      50, 0, 0, 0
  ])
  while True:
    pass

try:
  app()
except KeyboardInterrupt:
  print("KeyboardInterrupt")
  platform.stop()
  sys.exit()
  
