import PositionWatcher as w
import math
import time
import sys

instance = w.PositionWatcher()
def app():
  instance.start(False)
  while True:
    print(instance.getTicks())
    time.sleep(0.1)

try:
  app()
except KeyboardInterrupt:
  print("")
  print("KeyboardInterrupt")
  instance.stop()
  sys.exit()