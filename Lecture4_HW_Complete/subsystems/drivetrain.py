import math
import navx
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState, SwerveDrive4Kinematics
import wpimath.units

import constants
import ntutil
from hardware.swervemodule import SwerveModule


class Drivetrain:
    """
    The Drivetrain subsystem contains logic related to driving the robot around
    the field.
    """

    def __init__(self):
        # Hardware
        self.frontLeftSwerveModule = SwerveModule(25, 21, -3 * math.pi/2)
        self.frontRightSwerveModule = SwerveModule(28, 22, 0)
        self.backLeftSwerveModule = SwerveModule(26, 24, -math.pi)
        self.backRightSwerveModule = SwerveModule(27, 23, -math.pi/2)
        self.gyro = navx.AHRS.create_spi()

        # Swerve kinematics
        self.kinematics = SwerveDrive4Kinematics(
            constants.swerveModulePositions[0],
            constants.swerveModulePositions[1],
            constants.swerveModulePositions[2],
            constants.swerveModulePositions[3],
        )
        self.desiredChassisSpeeds = ChassisSpeeds()

        # NetworkTables topics
        nt = ntutil.getFolder("Drivetrain")
        self.desiredChassisSpeedsTopic = nt.getStructTopic("DesiredChassisSpeeds", ChassisSpeeds)
        self.desiredStatesTopic = nt.getStructArrayTopic("DesiredSwerveStates", SwerveModuleState)
        self.actualStatesTopic = nt.getStructArrayTopic("ActualSwerveStates", SwerveModuleState)

    def periodic(self):
        """
        Runs periodic logic specific to this subsystem. Should be called from
        robotPeriodic().
        """
        # Compute desired swerve module states and apply to swerve modules
        frontLeft, frontRight, backLeft, backRight = self.kinematics.toSwerveModuleStates(self.desiredChassisSpeeds)
        self.frontLeftSwerveModule.setDesiredState(frontLeft)
        self.frontRightSwerveModule.setDesiredState(frontRight)
        self.backLeftSwerveModule.setDesiredState(backLeft)
        self.backRightSwerveModule.setDesiredState(backRight)

        # Update NetworkTables topic
        self.desiredChassisSpeedsTopic.set(self.desiredChassisSpeeds)
        self.desiredStatesTopic.set([frontLeft, frontRight, backLeft, backRight])
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
        """
        Tells the drivetrain subsystem to move the robot at the desired speeds.
        All values are in field coordinates, not robot coordinates.
        """
        self.desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed,
            ySpeed,
            turnSpeed,
            self.gyro.getRotation2d(),
        )
