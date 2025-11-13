import rev


class Drivetrain:
  def __init__(self):
    self.leftMotor = rev.SparkMax(10, rev.SparkLowLevel.MotorType.kBrushless)
    self.rightMotor = rev.SparkMax(11, rev.SparkLowLevel.MotorType.kBrushless)

  def drive(self, leftSpeed: float, rightSpeed: float):
    self.leftMotor.set(leftSpeed)
    self.rightMotor.set(rightSpeed)
