import rev

# =============================================================================
# This file contains the Drivetrain class, which is dedicated to controlling
# the driving-related features of the robot. We call this a "subsystem"; a real
# FRC program will usually contain several "subsystem" files dedicated to
# different mechanisms of the robot, e.g. drivetrain, ball shooter, cone
# grabber, or elevator.
# =============================================================================

class Drivetrain:
  # HOMEWORK: Complete the __init__ method, which is a special function that
  # Python will run when creating an instance of Drivetrain, as you will in
  # robot.py. In it, create the following two instance variables:
  #  - self.leftMotor
  #  - self.rightMotor
  # 
  # Both motors should be an instance of `rev.SparkMax`, with a device ID of
  # your choosing, and motor type `rev.SparkLowLevel.MotorType.kBrushless`. If
  # you don't remember how to create them, go back and watch the recording!
  def __init__(self):
    self.leftMotor = rev.SparkMax(10, rev.SparkLowLevel.MotorType.kBrushless)
    self.rightMotor = rev.SparkMax(11, rev.SparkLowLevel.MotorType.kBrushless)

  # HOMEWORK: Complete the drive method, which is a custom function used to set
  # the left and right motor speeds. In it, you should call the `set` method on
  # both self.leftMotor and self.rightMotor, passing in the `leftSpeed` and
  # `rightSpeed` parameters.
  def drive(self, leftSpeed: float, rightSpeed: float):
    self.leftMotor.set(leftSpeed)
    self.rightMotor.set(rightSpeed)
