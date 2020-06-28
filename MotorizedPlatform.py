from time import sleep

'''
Abstration of motorized platform
'''
class MotorizedPlatform:
  # escSlots = {
  #   'frontLeft': 15,
  #   'frontRight': 12, # 12
  #   'backLeft': 14,
  #   'backRight': 13 # 13
  # }
  escSlots = [15,12,14,13]
  pwmInterface = None
  
  def __init__(self, pwmInterface = None):
    self.pwmInterface = pwmInterface
    if self.pwmInterface != None:
      self.pwmInterface.set_pwm_freq(50)
      for slot in self.escSlots:
        # 307 est le signal neutre sous 50 Hz (1.5 / 20 x 4096 = 307)
        self.pwmInterface.set_pwm(slot, 0, 307)
    else:
      print('> Motorized platform is mocked!!')
      
  def setSpeed(self, values):
    for i in range(len(values)):
      self.pwmInterface.set_pwm(
        self.escSlots[i],
        0,
        self.convertSpeedToEsc(values[i])
      )
    #print(arrayLabelValue)
    # if self.pwmInterface != None:
    #   for label in arrayLabelValue:
    #     self.pwmInterface.set_pwm(
    #       self.escSlots[label],
    #       0, 
    #       self.convertSpeedToEsc(arrayLabelValue[label])
    #     )
    # else:
    #   print(arrayLabelValue)

  # équivalent de la fonction map() de arduino
  def mappyt(self, x, inMin, inMax, outMin, outMax):
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin
  
  # fonction esc pour une vitesse de moteur de -100 à 100()
  def convertSpeedToEsc(self, speed):
    return round(self.mappyt(speed, 0, 100, 307, 410))
  
  def eastTranslation(self, speed):
    self.setSpeed({
      'frontLeft': speed, 'frontRight': speed,
      'backLeft':  speed,  'backRight': speed
    })
  
  def southTranslation(self, speed):
    self.northTranslation(-speed)

  def northTranslation(self, speed):
    self.setSpeed({
      'frontLeft': speed, 'frontRight': -speed,
      'backLeft': -speed, 'backRight':  speed
    })

  def westTranslation(self, speed):
    self.eastTranslation(- speed)

  def clockwiseRotation(self, speed):
    a = self.convertSpeedToEsc(speed)
    r = self.convertSpeedToEsc(-speed)
    self.setSpeed({
      'frontLeft': a,
      'frontRight': a,
      'backLeft': r,
      'backRight': r
    })
    # self.servo.set_pwm(self.escSlots['frontLeft'], 0, a)
    # self.servo.set_pwm(self.escSlots['frontRight'], 0, r)
    # self.servo.set_pwm(self.escSlots['backLeft'], 0, a)
    # self.servo.set_pwm(self.escSlots['backRight'], 0, r)

  def antiClockwiseRotation(self, speed):
    self.clockwiseRotation(-speed)

  def northEastTranslation(self, speed):
    a = self.convertSpeedToEsc(speed)
    s = self.convertSpeedToEsc(0)
    self.setSpeed({
      'frontLeft': a,
      'frontRight': s,
      'backLeft': s,
      'backRight': a
    })

  def southWestTranslation(self, speed):
    self.northEastTranslation(-speed)

  def northWestTranslation(self, speed):
    self.setSpeed({
      'frontLeft': 0,
      'frontRight': speed,
      'backLeft': 0,
      'backRight': speed
    })

  def southEastTranslation(self, speed):
    self.northWestTranslation(-speed)

  def stop(self):
    #print('STOP ALL')
    # self.setSpeed({
    #   'frontLeft': 0,
    #   'frontRight': 0,
    #   'backLeft': 0,
    #   'backRight': 0
    # })
    self.setSpeed([0, 0, 0, 0])
  