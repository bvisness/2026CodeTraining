import math
from wpimath.kinematics import SwerveModuleState

import ntutil
from hardware.swervemodule import SwerveModule


class Drivetrain:
  def __init__(self):
    self.frontLeftSwerveModule = SwerveModule(25, 21, 3 * math.pi/2)
    self.frontRightSwerveModule = SwerveModule(28, 22, 0)
    self.backLeftSwerveModule = SwerveModule(26, 24, math.pi)
    self.backRightSwerveModule = SwerveModule(27, 23, math.pi/2)

    self.nt = ntutil.folder("Drivetrain")
    self.swerveStatesTopic = self.nt.getStructArrayTopic("ActualSwerveStates", SwerveModuleState)
  
  def periodic(self):
    self.swerveStatesTopic.set([
      self.frontLeftSwerveModule.getState(),
      self.frontRightSwerveModule.getState(),
      self.backLeftSwerveModule.getState(),
      self.backRightSwerveModule.getState(),
    ])
