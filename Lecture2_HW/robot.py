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
    # HOMEWORK: In robotInit, create the following two instance variables:
    #  - self.arm (set to an instance of the Arm subsystem)
    #  - self.gamepad (a joystick on port 0 like usual)
    def robotInit(self):
        # Code goes here...
        pass

    # HOMEWORK: In robotPeriodic, which runs 50 times per second whether or not
    # the robot is enabled, call the `periodic` method of self.arm.
    def robotPeriodic(self):
        # Code goes here...
        pass

    # HOMEWORK: In teleopPeriodic, add controls for manipulating the arm. It
    # should work like so:
    #  - When the left stick is at rest, the arm should be all the way down (at
    #    an angle of 0 radians).
    #  - When the left stick is all the way forward, the arm should be all the
    #    way up (at an angle of pi/2 radians).
    #  - The intake wheel attached to the arm should spin if the A button is
    #    being pressed, otherwise the intake wheels should stop.
    # You can verify if the controls are working correctly by looking at the
    # Mechanism tab in AdvantageScope like we discussed in class.
    def teleopPeriodic(self):
        # Code goes here...
        pass
