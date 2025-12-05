import wpilib
import wpimath

from subsystems.drivetrain import Drivetrain


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.drivetrain = Drivetrain()
        self.gamepad = wpilib.Joystick(0)

    def robotPeriodic(self):
        self.drivetrain.periodic()

    def teleopPeriodic(self):
        xSpeed = wpimath.applyDeadband(-self.gamepad.getRawAxis(1), 0.1) * 2
        ySpeed = wpimath.applyDeadband(-self.gamepad.getRawAxis(0), 0.1) * 2
        turnSpeed = wpimath.applyDeadband(-self.gamepad.getRawAxis(4), 0.1) * 2
        self.drivetrain.drive(xSpeed, ySpeed, turnSpeed)
