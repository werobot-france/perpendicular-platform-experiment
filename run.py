import PositionWatcher as w
import math
import time

def onChange(x, y, theta):
  print(x, y, math.degrees(theta))
  
instance = w.PositionWatcher()
#instance.setOnPositionChangedHandler(onChange)
instance.start()

while (True):
    print(instance.getPosRot())
    time.sleep(0.2)
