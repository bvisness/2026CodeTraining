import wpilib

from subsystems.drivetrain import Drivetrain


class MyRobot(wpilib.TimedRobot):
  def robotInit(self):
    self.drivetrain = Drivetrain()
    self.gamepad = wpilib.Joystick(0)

  def disabledInit(self):
    self.drivetrain.drive(0, 0)

  def teleopPeriodic(self):
    self.drivetrain.drive(-self.gamepad.getRawAxis(1), -self.gamepad.getRawAxis(3))
