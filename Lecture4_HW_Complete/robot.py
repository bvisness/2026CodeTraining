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
        xSpeed = wpimath.applyDeadband(-self.gamepad.getRawAxis(1), 0.1) * 4.5
        ySpeed = wpimath.applyDeadband(-self.gamepad.getRawAxis(0), 0.1) * 4.5
        turnSpeed = wpimath.applyDeadband(-self.gamepad.getRawAxis(4), 0.1) * 2 * 3.14159
        self.drivetrain.drive(xSpeed, ySpeed, turnSpeed)
