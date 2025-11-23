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

    self.drivePidController = self.driveMotor.getClosedLoopController()
    self.steerPidController = self.steerMotor.getClosedLoopController()
  
  def setState(self, state: SwerveModuleState):
    state.angle += Rotation2d(self.angleOffset)
    encoderRotation = Rotation2d(self.steerAbsoluteEncoder.getPosition())
    state.optimize(encoderRotation)
    state.cosineScale(encoderRotation)
    self.drivePidController.setReference(state.speed, rev.SparkLowLevel.ControlType.kVelocity)
    self.steerPidController.setReference(state.angle.radians(), rev.SparkLowLevel.ControlType.kPosition)

  def getState(self) -> SwerveModuleState:
    return SwerveModuleState(
      self.driveEncoder.getVelocity(),
      Rotation2d(self.steerAbsoluteEncoder.getPosition() - self.angleOffset)
    )
  
  def getPosition(self) -> SwerveModulePosition:
    return SwerveModulePosition(
      self.driveEncoder.getPosition(),
      Rotation2d(self.steerAbsoluteEncoder.getPosition() - self.angleOffset)
    )
