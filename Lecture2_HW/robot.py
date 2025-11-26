import wpilib

import constants
import math
from subsystems.arm import Arm
import utils


# =============================================================================
# As usual, this file contains the main robot class. This is where you read
# from joysticks and send commands to the robot's subsystems. This project
# contains one subsystem, called Arm.
# =============================================================================

class MyRobot(wpilib.TimedRobot):
    # HOMEWORK: In robotInit, create the following two instance variables:
    #  - self.arm (set to an instance of the Arm subsystem)
    #  - self.gamepad (a joystick on port 0 like usual)
    def robotInit(self):
        # Code goes here...
        pass

    # HOMEWORK: In robotPeriodic, call the `periodic` method of self.arm to
    # ensure that subsystem logic is running once per frame.
    def robotPeriodic(self):
        # Code goes here...
        pass

    # Teleop controls for the arm. This will not work until you have defined
    # self.arm and self.gamepad correctly in robotInit.
    def teleopPeriodic(self):
        leftStick = self.gamepad.getRawAxis(1)
        desiredAngle = utils.remap(leftStick, (-1, 1), (math.pi, 0))
        self.arm.setDesiredArmAngle(desiredAngle)

        intakeSpeed = 0.5 if self.gamepad.getRawButton(1) else 0
        self.arm.setIntakeSpeed(intakeSpeed)
