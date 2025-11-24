import wpilib

import constants
from subsystems.arm import Arm
import utils


# =============================================================================
# As usual, this file contains the main robot class. This is where you read
# from joysticks and send commands to the robot's subsystems. This project
# contains one subsystem, called Arm.
# =============================================================================

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.arm = Arm()
        self.gamepad = wpilib.Joystick(0)

    def robotPeriodic(self):
        self.arm.periodic()

    def teleopPeriodic(self):
        armAngle = utils.remap(
            self.gamepad.getRawAxis(1),
            (0, -1), # Remember, -1 is up on the left stick.
            (constants.armMinAngle, constants.armMaxAngle),
        )
        self.arm.setDesiredArmAngle(armAngle)

        if self.gamepad.getRawButton(1):
            intakeSpeed = 0.5
        else:
            intakeSpeed = 0
        self.arm.setIntakeSpeed(intakeSpeed)
