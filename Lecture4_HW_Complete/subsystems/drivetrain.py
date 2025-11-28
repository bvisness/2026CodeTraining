import math
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import SwerveModuleState, SwerveDrive4Kinematics

import constants
import ntutil
from hardware.swervemodule import SwerveModule


class Drivetrain:
    def __init__(self):
        self.frontLeftSwerveModule = SwerveModule(25, 21, 3 * math.pi/2)
        self.frontRightSwerveModule = SwerveModule(28, 22, 0)
        self.backLeftSwerveModule = SwerveModule(26, 24, math.pi)
        self.backRightSwerveModule = SwerveModule(27, 23, math.pi/2)

        wd = constants.wheelDistanceFromCenter
        self.kinematics = SwerveDrive4Kinematics(
            Translation2d(wd, wd),
            Translation2d(wd, -wd),
            Translation2d(-wd, wd),
            Translation2d(-wd, -wd),
        )

        self.nt = ntutil.folder("Drivetrain")
        self.actualStatesTopic = self.nt.getStructArrayTopic("ActualSwerveStates", SwerveModuleState)

    def periodic(self):
        self.actualStatesTopic.set([
            self.frontLeftSwerveModule.getState(),
            self.frontRightSwerveModule.getState(),
            self.backLeftSwerveModule.getState(),
            self.backRightSwerveModule.getState(),
        ])
