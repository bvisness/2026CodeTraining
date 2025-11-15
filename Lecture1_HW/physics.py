import pyfrc.physics.core
from pyfrc.physics.core import PhysicsInterface
from rev import SparkMaxSim
import wpilib
from wpilib import RobotController
from wpilib.simulation import DifferentialDrivetrainSim
from wpimath import units
from wpimath.geometry import Pose2d
from wpimath.system.plant import DCMotor

import ntutil
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
    # If setup succeeds, we will set this to true.
    self.ok = False

    # NetworkTables topics
    self.poseTopic = ntutil.getStructTopic("Pose", Pose2d)

    # Fire alerts for common problems and ensure that the sim doesn't crash on startup
    self.badDrivetrainAlert = wpilib.Alert("No instance variable named 'drivetrain' in MyRobot; make sure you created it in robotInit", wpilib.Alert.AlertType.kError)
    self.badLeftMotorAlert = wpilib.Alert("No instance variable named 'leftMotor' in Drivetrain; make sure you created it in __init__", wpilib.Alert.AlertType.kError)
    self.badRightMotorAlert = wpilib.Alert("No instance variable named 'rightMotor' in Drivetrain; make sure you created it in __init__", wpilib.Alert.AlertType.kError)
    try:
      robot.drivetrain
    except AttributeError:
      self.badDrivetrainAlert.set(True)
      return
    try:
      robot.drivetrain.leftMotor
    except AttributeError:
      self.badLeftMotorAlert.set(True)
      return
    try:
      robot.drivetrain.rightMotor
    except AttributeError:
      self.badRightMotorAlert.set(True)
      return

    # Physics device setup
    self.robot = robot
    self.leftDriveMotor = SparkMaxSim(robot.drivetrain.leftMotor, DCMotor.NEO(1))
    self.rightDriveMotor = SparkMaxSim(robot.drivetrain.rightMotor, DCMotor.NEO(1))
    self.drivetrainSim = DifferentialDrivetrainSim.createKitbotSim(
      DCMotor.NEO(1), # one NEO per side
      10.71, # 10.71:1 gearing
      units.inchesToMeters(4), # 4-inch wheels
    )

    self.ok = True


  def update_sim(self, now: float, tm_diff: float):
    # Early exit if setup failed earlier.
    if not self.ok:
      return

    vBus = RobotController.getBatteryVoltage()
    if self.robot.isEnabled():
      self.drivetrainSim.setInputs(
        self.leftDriveMotor.getAppliedOutput() * vBus,
        self.rightDriveMotor.getAppliedOutput() * vBus
      )
    else:
      self.drivetrainSim.setInputs(0, 0)

    self.drivetrainSim.update(kDeltaTime)
    self.leftDriveMotor.iterate(
      self.drivetrainSim.getLeftVelocity(),
      vBus,
      kDeltaTime,
    )
    self.rightDriveMotor.iterate(
      self.drivetrainSim.getRightVelocity(),
      vBus,
      kDeltaTime,
    )

    self.poseTopic.set(self.drivetrainSim.getPose())
