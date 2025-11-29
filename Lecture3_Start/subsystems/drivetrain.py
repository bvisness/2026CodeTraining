import wpimath.units
from wpimath.geometry import Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState

from hardware.swervemodule import SwerveModule
import ntutil


class Drivetrain:
    def __init__(self):
        # TODO: Create frontLeftModule, frontRightModule, backLeftModule, backRightModule
        # Example:
        #   self.frontLeftModule = SwerveModule(1, 2, 0)
        ...
        
        # TODO: Create kinematics object for calculating swerve module states
        self.kinematics = ...

        # TODO: Use this variable to set the desired states of all the swerve modules
        self.desiredChassisSpeeds = ChassisSpeeds()

        # Create NetworkTables topics to show in AdvantageScope
        nt = ntutil.folder("Drivetrain")
        self.desiredChassisSpeedsTopic = nt.getStructTopic("DesiredChassisSpeeds", ChassisSpeeds)
        self.desiredStatesTopic = nt.getStructArrayTopic("DesiredSwerveStates", SwerveModuleState)
        self.actualStatesTopic = nt.getStructArrayTopic("ActualSwerveStates", SwerveModuleState)

    def periodic(self):
        # TODO: Compute swerve module states using kinematics
        ...

        # Report actual/desired swerve info
        self.desiredChassisSpeedsTopic.set(self.desiredChassisSpeeds)
        # self.desiredStatesTopic.set([frontLeft, frontRight, backLeft, backRight])
        # self.actualStatesTopic.set([
        #     self.frontLeftModule.getActualState(),
        #     self.frontRightModule.getActualState(),
        #     self.backLeftModule.getActualState(),
        #     self.backRightModule.getActualState(),
        # ])
    
    def drive(
        self,
        xSpeed: wpimath.units.meters_per_second,
        ySpeed: wpimath.units.meters_per_second,
        turnSpeed: wpimath.units.radians_per_second,
    ):
        # TODO: Set self.desiredChassisSpeeds
        ...
