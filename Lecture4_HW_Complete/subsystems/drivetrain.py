import math
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState, SwerveDrive4Kinematics
import wpimath.units

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
        self.desiredChassisSpeeds = ChassisSpeeds()

        self.nt = ntutil.folder("Drivetrain")
        self.desiredChassisSpeedsTopic = self.nt.getStructTopic("DesiredChassisSpeeds", ChassisSpeeds)
        self.desiredStatesTopic = self.nt.getStructArrayTopic("DesiredSwerveStates", SwerveModuleState)
        self.actualStatesTopic = self.nt.getStructArrayTopic("ActualSwerveStates", SwerveModuleState)

    def periodic(self):
        # Compute desired swerve module states and apply to swerve modules
        frontLeft, frontRight, backLeft, backRight = self.kinematics.toSwerveModuleStates(self.desiredChassisSpeeds)
        self.frontLeftSwerveModule.setDesiredState(frontLeft)
        self.frontRightSwerveModule.setDesiredState(frontRight)
        self.backLeftSwerveModule.setDesiredState(backLeft)
        self.backRightSwerveModule.setDesiredState(backRight)

        # Report actual/desired swerve info
        self.desiredChassisSpeedsTopic.set(self.desiredChassisSpeeds)
        self.desiredStatesTopic.set([
            frontLeft,
            frontRight,
            backLeft,
            backRight,
        ])
        self.actualStatesTopic.set([
            self.frontLeftSwerveModule.getActualState(),
            self.frontRightSwerveModule.getActualState(),
            self.backLeftSwerveModule.getActualState(),
            self.backRightSwerveModule.getActualState(),
        ])

    def drive(
        self,
        xSpeed: wpimath.units.meters_per_second,
        ySpeed: wpimath.units.meters_per_second,
        turnSpeed: wpimath.units.radians_per_second,
    ):
        # TODO: Get heading from navX
        self.desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, Rotation2d())
