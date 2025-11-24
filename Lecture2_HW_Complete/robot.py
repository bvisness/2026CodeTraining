import math
import wpilib
from wpimath.system.plant import DCMotor

import constants
import ntutil
from subsystems.arm import Arm
import utils


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.arm = Arm()
        self.gamepad = wpilib.Joystick(0)

        ntutil.log(f"neo stall torque: {DCMotor.NEO(1).stallTorque}")

    def robotPeriodic(self):
        self.arm.periodic()

    def teleopPeriodic(self):
        armAngle = utils.remap(
            self.gamepad.getRawAxis(1),
            (0, -1), # Remember, -1 is up on the left stick.
            (constants.kArmMinAngle, constants.kArmMaxAngle),
        )
        self.arm.setDesiredArmAngle(armAngle)

        if self.gamepad.getRawButton(1):
            intakeSpeed = 0.5
        else:
            intakeSpeed = 0
        self.arm.intakeMotor.set(intakeSpeed)
