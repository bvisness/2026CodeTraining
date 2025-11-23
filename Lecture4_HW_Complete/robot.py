import wpilib

from subsystems.drivetrain import Drivetrain


class MyRobot(wpilib.TimedRobot):
  def robotInit(self):
    self.drivetrain = Drivetrain()
    self.gamepad = wpilib.Joystick(0)

  def robotPeriodic(self):
    self.drivetrain.periodic()

  def teleopPeriodic(self):
    self.drivetrain.frontLeftSwerveModule.driveMotor.set(-self.gamepad.getRawAxis(1))
