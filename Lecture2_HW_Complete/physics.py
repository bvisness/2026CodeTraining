import math
import pyfrc.physics.core
from pyfrc.physics.core import PhysicsInterface
import random
from rev import SparkMaxSim
from wpilib import DriverStation, RobotController
import wpimath
import wpimath.units
from wpimath.controller import PIDController
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveModuleState
from wpimath.system.plant import DCMotor

import ntutil
from hardware.swervemodule import SwerveModule
from robot import MyRobot

# =============================================================================
# This file controls the physics simulation used when running in the WPILib
# simulator. You should NOT need to touch it for the homework.
# =============================================================================

# Our robot program always ticks at 50Hz
kDeltaTime = 1.0 / 50

# See https://robotpy.readthedocs.io/projects/pyfrc/en/stable/physics.html
class PhysicsEngine(pyfrc.physics.core.PhysicsEngine):
  def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
    # NetworkTables topics
    simFolder = ntutil.folder("Sim")
    self.poseTopic = simFolder.getStructTopic("Pose", Pose2d)

    # Physics device setup
    self.robot = robot
    frontLeft = SwerveModuleSim(robot.drivetrain.frontLeftSwerveModule)
    frontRight = SwerveModuleSim(robot.drivetrain.frontRightSwerveModule)
    backLeft = SwerveModuleSim(robot.drivetrain.backLeftSwerveModule)
    backRight = SwerveModuleSim(robot.drivetrain.backRightSwerveModule)
    self.drivetrain = SwerveDriveSim(frontLeft, frontRight, backLeft, backRight)

  def update_sim(self, now: float, tm_diff: float):
    vbus = RobotController.getBatteryVoltage()

    self.drivetrain.update_sim(vbus, tm_diff)    

    self.poseTopic.set(self.drivetrain.get_pose())
    # self.swerveStatesTopic.set(list(self.drivetrain.get_module_states()))


class SwerveModuleSim:
  def __init__(self, module: SwerveModule):
    self.driveMotorSim = SparkMaxSim(module.driveMotor, DCMotor.NEO(1))
    self.steerMotorSim = SparkMaxSim(module.steerMotor, DCMotor.NEO(1))
    self.steerEncoderSim = self.steerMotorSim.getAbsoluteEncoderSim()
    self.angleOffset = module.angleOffset

    # Calculates the velocity of the drive motor, roughly simulating
    # inertia and friction. Note that this limits the SPEED of the motors.
    # Linear change to velocity means constant acceleration, so the units
    # here check out.
    self.driveSpeedLimiter = SlewRateLimiter(9.8 * 1) # 1 G (m/s^2)

    # Use a PID controller to simulate velocity of the steer motor
    self.steerController = PIDController(100, 0, 0) # magic numbers!
    self.steerController.enableContinuousInput(-math.pi, math.pi)

    # Randomize the starting rotation to simulate what we have when we take
    # the field
    self.steerMotorSim.setPosition(random.uniform(-math.pi, math.pi))

  def update_sim(self, vbus: float, tm_diff: float):
    targetSpeed = self.driveMotorSim.getSetpoint() if DriverStation.isEnabled() else 0
    driveSpeed = self.driveSpeedLimiter.calculate(targetSpeed)
    self.driveMotorSim.iterate(driveSpeed, vbus, tm_diff)

    targetAngle = self.steerMotorSim.getSetpoint()
    currentAngle = wpimath.angleModulus(self.steerMotorSim.getPosition())
    self.steerController.setSetpoint(targetAngle)
    steerSpeed = self.steerController.calculate(currentAngle)
    self.steerMotorSim.iterate(1, vbus, tm_diff) # TODO: 1 mystery unit per second?
    self.steerEncoderSim.iterate(1, tm_diff) # TODO: Translate from steer motor velocity into encoder angle (should be straightforward gearing)
  
  def get_state(self) -> SwerveModuleState:
    return SwerveModuleState(
      wpimath.units.meters_per_second(self.driveMotorSim.getVelocity()),
      Rotation2d(self.steerEncoderSim.getPosition() - self.angleOffset)
    )


class SwerveDriveSim:
  def __init__(self, frontLeft: SwerveModuleSim, frontRight: SwerveModuleSim, backLeft: SwerveModuleSim, backRight: SwerveModuleSim):
    wd = wpimath.units.inchesToMeters(12.5) # wheel distance
    self.kinematics = SwerveDrive4Kinematics(
      Translation2d(wd, wd),
      Translation2d(wd, -wd),
      Translation2d(-wd, wd),
      Translation2d(-wd, -wd),
    )
    self.modules = (frontLeft, frontRight, backLeft, backRight)

    # # Per the navX docs, the recommended way to use the navX as a sim
    # # device is to just use the real device but set the simulator variable
    # # for Yaw directly.
    # #
    # # https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/c/
    # navXDevice = hal.simulation.getSimDeviceHandle(f"navX-Sensor[{drivetrain.gyro.getPort()}]")
    # self.navXAngleSim = hal.SimDouble(hal.simulation.getSimValueHandle(navXDevice, "Yaw"))

    self.pose = Pose2d()
  
  def update_sim(self, vbus: float, tm_diff: float):
    for module in self.modules:
      module.update_sim(vbus, tm_diff)
    moduleStates = self.get_module_states()
    chassisSpeeds = self.kinematics.toChassisSpeeds(moduleStates)
    self.pose = self.pose.exp(chassisSpeeds.toTwist2d(tm_diff))

    # # Update the simulated gyro. For reasons unknown, the navX uses
    # # clockwise degrees.
    # self.navXAngleSim.set(-self.pose.rotation().degrees())

  def get_module_states(self):
    return (
      self.modules[0].get_state(),
      self.modules[1].get_state(),
      self.modules[2].get_state(),
      self.modules[3].get_state(),
    )

  def get_pose(self):
    return self.pose

  def set_pose(self, pose: Pose2d):
    self.pose = pose
