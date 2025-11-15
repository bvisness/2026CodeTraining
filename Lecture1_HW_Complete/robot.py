import wpilib

from subsystems.drivetrain import Drivetrain

# =============================================================================
# The robot.py file contains your main robot class, here called MyRobot. The
# robot class defines functions that RobotPy will run at different times during
# the course of a match.
# =============================================================================

class MyRobot(wpilib.TimedRobot):
  # HOMEWORK: Complete the robotInit method, which runs once at robot startup.
  # In it, create the following two instance variables:
  #  - self.drivetrain
  #  - self.gamepad
  def robotInit(self):
    self.drivetrain = Drivetrain()
    self.gamepad = wpilib.Joystick(0)

  # HOMEWORK: Complete the teleopPeriodic method, which runs 50 times per
  # second when teleop is enabled. In it, you should:
  #  - Get the current values of the left and right sticks on self.gamepad
  #  - Pass them to the `drive` method of self.drivetrain to control the left
  #    and right wheels.
  def disabledInit(self):
    self.drivetrain.drive(0, 0)

  # HOMEWORK: Add the disabledInit method, which runs once each time the robot
  # enters the "disabled" mode. In it, you should:
  #  - Call the `drive` method of self.drivetrain with zero speed to stop both
  #    the left and right motors.
  def teleopPeriodic(self):
    self.drivetrain.drive(-self.gamepad.getRawAxis(1), -self.gamepad.getRawAxis(3))
