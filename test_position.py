import PositionWatcher as w
import math
import time
import sys

def onChange(x, y, theta, tb):
  print(round(x, 0), round(y, 0), round(math.degrees(theta), 0), tb)

instance = w.PositionWatcher()
  
def app():
  instance.setOnPositionChangedHandler(onChange)
  instance.start()

try:
  app()
except KeyboardInterrupt:
  print("")
  print("KeyboardInterrupt")
  instance.stop()
  sys.exit()