from Navigation import Navigation
from math import pi

nav = Navigation(None, None)

'''
assertion 

'''

# theta = 0 - north
print(nav.getSpeedFromAngle(0, 10)) 

# theta = pi/2 - west
print(nav.getSpeedFromAngle(pi/2, 10)) 

# theta = pi/2 - south
print(nav.getSpeedFromAngle(pi, 10)) 