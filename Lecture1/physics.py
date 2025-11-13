import pyfrc.physics.core
from pyfrc.physics.core import PhysicsInterface
from rev import SparkMaxSim
from wpilib import RobotController
from wpilib.simulation import DifferentialDrivetrainSim
from wpimath import units
from wpimath.geometry import Pose2d
from wpimath.system.plant import DCMotor

import ntutil
from robot import MyRobot

# See https://robotpy.readthedocs.io/projects/pyfrc/en/stable/physics.html

# Our robot program always ticks at 50Hz
kDeltaTime = 1.0 / 50

class PhysicsEngine(pyfrc.physics.core.PhysicsEngine):
  def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
    # Physics device setup
    self.robot = robot
    self.leftDriveMotor = SparkMaxSim(robot.drivetrain.leftMotor, DCMotor.NEO(1))
    self.rightDriveMotor = SparkMaxSim(robot.drivetrain.rightMotor, DCMotor.NEO(1))
    self.drivetrainSim = DifferentialDrivetrainSim.createKitbotSim(
      DCMotor.NEO(1), # one NEO per side
      10.71, # 10.71:1 gearing
      units.inchesToMeters(4), # 4-inch wheels
    )

    # NetworkTables topics
    simFolder = ntutil.folder("Sim")
    self.poseTopic = simFolder.getStructTopic("Pose", Pose2d)

  def update_sim(self, now: float, tm_diff: float):
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
