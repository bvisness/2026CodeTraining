import rev
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

import configs


class SwerveModule:
    def __init__(self, driveMotorId: int, steerMotorId: int, angleOffset: float):
        self.driveMotor = rev.SparkMax(driveMotorId, rev.SparkLowLevel.MotorType.kBrushless)
        self.steerMotor = rev.SparkMax(steerMotorId, rev.SparkLowLevel.MotorType.kBrushless)
        self.angleOffset: float = angleOffset

        self.driveMotor.configure(configs.driveMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
        self.steerMotor.configure(configs.steerMotorConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

        self.driveEncoder = self.driveMotor.getEncoder()
        self.steerRelativeEncoder = self.steerMotor.getEncoder()
        self.steerAbsoluteEncoder = self.steerMotor.getAbsoluteEncoder()

        self.driveController = self.driveMotor.getClosedLoopController()
        self.steerController = self.steerMotor.getClosedLoopController()

    def setDesiredState(self, state: SwerveModuleState):
        """
        Sets the desired state of the swerve module (angle/speed). This method
        will account for the swerve module's angle offset, so the angle
        provided should NOT be modified to account for angle offsets. (In other
        words, pass the raw SwerveModuleState straight out of kinematics.)
        """
        correctedAngle = state.angle + Rotation2d(self.angleOffset)
        # In real robot projects, we might do other work here to improve swerve
        # behavior, e.g. driving wheels more slowly when they are still pointing in
        # the wrong direction.
        self.driveController.setReference(state.speed, rev.SparkLowLevel.ControlType.kVelocity)
        self.steerController.setReference(correctedAngle.radians(), rev.SparkLowLevel.ControlType.kPosition)

    def getActualState(self) -> SwerveModuleState:
        """
        Gets the actual state of the swerve module (angle/speed), as reported
        by the NEO encoders.
        """
        return SwerveModuleState(
            self.driveEncoder.getVelocity(),
            Rotation2d(self.steerAbsoluteEncoder.getPosition() - self.angleOffset)
        )

    def getActualPosition(self) -> SwerveModulePosition:
        """
        Gets the actual position of the swerve module (drive/steer positions),
        as reported by the NEO encoders.
        """
        return SwerveModulePosition(
            self.driveEncoder.getPosition(),
            Rotation2d(self.steerAbsoluteEncoder.getPosition() - self.angleOffset)
        )
