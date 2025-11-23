import math
import wpilib
from wpimath.system.plant import DCMotor

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
        armAngle = utils.remap(self.gamepad.getRawAxis(1), (-1, 1), (math.pi, -math.pi))
        self.arm.setDesiredAngle(armAngle)
        tempArmSpeed = utils.remap(self.gamepad.getRawAxis(1), (-1, 1), (0.2, -0.2))
        self.arm.armMotor.set(tempArmSpeed)

        intakeSpeed = 0
        if self.gamepad.getRawButton(1):
            intakeSpeed = 0.5
        elif self.gamepad.getRawButton(2):
            intakeSpeed = -0.5
        self.arm.intakeMotor.set(intakeSpeed)
