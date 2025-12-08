import wpilib

from subsystems.drivetrain import Drivetrain


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.drivetrain = Drivetrain()
        self.gamepad = wpilib.Joystick(0)

    def robotPeriodic(self):
        self.drivetrain.periodic()

    def teleopPeriodic(self):
        xSpeed = -self.gamepad.getRawAxis(1) * 4
        ySpeed = -self.gamepad.getRawAxis(0) * 4
        turnSpeed = -self.gamepad.getRawAxis(4) * 2
        self.drivetrain.drive(xSpeed, ySpeed, turnSpeed)
