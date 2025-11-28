import math
import random
import typing

import pyfrc.physics.core
from pyfrc.physics.core import PhysicsInterface
from rev import SparkMax, SparkMaxSim
from wpilib import DriverStation, RobotController
from wpilib.simulation import BatterySim, RoboRioSim
import wpimath
import wpimath.units
from wpimath.controller import PIDController
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveModuleState
from wpimath.system.plant import DCMotor

import constants
from hardware.swervemodule import SwerveModule
import ntutil
from robot import MyRobot
import utils


# =============================================================================
# This file controls the physics simulation used when running in the WPILib
# simulator. You should NOT need to touch it for the homework.
# =============================================================================

# See https://robotpy.readthedocs.io/projects/pyfrc/en/stable/physics.html
class PhysicsEngine(pyfrc.physics.core.PhysicsEngine):
    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        # NetworkTables topics
        simFolder = ntutil.folder("Sim")
        self.poseTopic = simFolder.getStructTopic("Pose", Pose2d)
        battery = simFolder.folder("Battery")
        self.batteryVoltageTopic = battery.getFloatTopic("Voltage")
        self.batteryCurrentTopic = battery.getFloatTopic("Current")

        # Physics device setup
        self.robot = robot
        self.battery = DrainingBatterySim()
        self.drivetrain = SwerveDriveSim(
            robot.drivetrain.frontLeftSwerveModule,
            robot.drivetrain.frontRightSwerveModule,
            robot.drivetrain.backLeftSwerveModule,
            robot.drivetrain.backRightSwerveModule,
            robot.drivetrain.kinematics,
            nt=simFolder.folder("Drivetrain")
        )

    def update_sim(self, now: float, tm_diff: float):
        vbus = RobotController.getBatteryVoltage()

        self.drivetrain.update_sim(vbus, tm_diff)

        self.poseTopic.set(self.drivetrain.get_pose())
        # self.swerveStatesTopic.set(list(self.drivetrain.get_module_states()))

        # Update simulated electrical state
        # TODO: Our current calculations are currently returning abject
        # nonsense. For now we will just bypass current draw simulation.
        # currents = [self.drivetrain.get_current_draw()]
        currents = []
        self.battery.update_sim(sum(currents), tm_diff)
        RoboRioSim.setVInVoltage(self.battery.output_voltage())
        RoboRioSim.setVInCurrent(self.battery.output_current())
        self.batteryVoltageTopic.set(RobotController.getBatteryVoltage())
        self.batteryCurrentTopic.set(RobotController.getInputCurrent())


class SwerveModuleSim:
    def __init__(
        self,
        module: SwerveModule,
        *, nt: ntutil._NTFolder = ntutil._DummyNTFolder(),
    ):
        self.driveSparkSim = SparkMaxSim2175(
            module.driveMotor,
            DCMotor.NEO(1).withReduction(constants.driveMotorReduction),
            nt=nt.folder("DriveMotor"),
        )
        self.steerSparkSim = SparkMaxSim2175(
            module.steerMotor,
            DCMotor.NEO550(1).withReduction(constants.steerMotorReduction),
            nt=nt.folder("SteerMotor"),
        )
        self.steerAbsoluteEncoderSim = self.steerSparkSim.getAbsoluteEncoderSim()
        self.angleOffset = module.angleOffset

        # Randomize the starting rotation to simulate what we have when we take
        # the field
        self.steerSparkSim.setPosition(random.uniform(-math.pi, math.pi))

    def update_sim(self, vbus: float, tm_diff: float):
        self.driveSparkSim.iterate(1, vbus, tm_diff) # 1 m/s
        self.steerSparkSim.iterate(math.pi*2, vbus, tm_diff) # 1 rev/s in rad/s
        # targetSpeed = self.driveSparkSim.getSetpoint() if DriverStation.isEnabled() else 0
        # driveSpeed = self.driveSpeedLimiter.calculate(targetSpeed)
        # self.driveSparkSim.iterate(driveSpeed, vbus, tm_diff)

        # # TODO: redo all of this, it is Jank and Wrong
        # targetAngle = self.steerSparkSim.getSetpoint()
        # currentAngle = wpimath.angleModulus(self.steerSparkSim.getPosition())
        # self.steerController.setSetpoint(targetAngle)
        # steerSpeed = self.steerController.calculate(currentAngle)
        # self.steerSparkSim.iterate(1, vbus, tm_diff) # TODO: 1 mystery unit per second?
        # self.steerAbsoluteEncoderSim.iterate(1, tm_diff) # TODO: Translate from steer motor velocity into encoder angle (should be straightforward gearing)

    def get_state(self) -> SwerveModuleState:
        return SwerveModuleState(
            wpimath.units.meters_per_second(self.driveSparkSim.getVelocity()),
            Rotation2d(self.steerAbsoluteEncoderSim.getPosition() - self.angleOffset)
        )

    def get_current_draw(self) -> wpimath.units.amperes:
        return self.driveSparkSim.getMotorCurrent() + self.steerSparkSim.getMotorCurrent()


class SwerveDriveSim:
    def __init__(
        self,
        frontLeft: SwerveModule,
        frontRight: SwerveModule,
        backLeft: SwerveModule,
        backRight: SwerveModule,
        kinematics: SwerveDrive4Kinematics,
        *, nt: ntutil._NTFolder = ntutil._DummyNTFolder(),
    ):
        # TODO: Get from constants
        wd = wpimath.units.inchesToMeters(12.5) # wheel distance
        self.modules: tuple[SwerveModuleSim, SwerveModuleSim, SwerveModuleSim, SwerveModuleSim] = (
            SwerveModuleSim(frontLeft, nt=nt.folder("FrontLeft")),
            SwerveModuleSim(frontRight, nt=nt.folder("FrontRight")),
            SwerveModuleSim(backLeft, nt=nt.folder("BackLeft")),
            SwerveModuleSim(backRight, nt=nt.folder("BackRight")),
        )
        self.kinematics = kinematics

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

    def get_current_draw(self) -> wpimath.units.amperes:
        current = sum(s.get_current_draw() for s in self.modules)
        return current


class DrainingBatterySim():
    """
    An extended version of the WPILib BatterySim that simulates typical battery
    drain over the course of a match.
    """

    # Empirical good-enough voltage values for an FRC battery as it drains.
    MAX_OUTPUT_VOLTS: wpimath.units.volts = 12.9
    MIN_OUTPUT_VOLTS: wpimath.units.volts = 10.5

    # Less than the nominal amp-hours of 18, for Realism.
    FULL_BATTERY_AMP_HOURS: wpimath.units.ampere_hours = 12

    def __init__(self, *, usable_amp_hours: wpimath.units.ampere_hours | None = None):
        self.charge: wpimath.units.ampere_hours = DrainingBatterySim.FULL_BATTERY_AMP_HOURS if usable_amp_hours is None else usable_amp_hours
        self.currentDraw: wpimath.units.amperes = 0

    def update_sim(self, current_draw: wpimath.units.amperes, tm_diff: float):
        spentCharge: wpimath.units.ampere_hours = current_draw * (tm_diff / 60 / 60)
        self.currentDraw = current_draw
        self.charge = max(0, self.charge - spentCharge)

    def nominal_voltage(self) -> wpimath.units.volts:
        return utils.remap(
            self.charge,
            (DrainingBatterySim.FULL_BATTERY_AMP_HOURS, 0),
            (DrainingBatterySim.MAX_OUTPUT_VOLTS, DrainingBatterySim.MIN_OUTPUT_VOLTS),
        )

    def output_voltage(self) -> wpimath.units.volts:
        return BatterySim.calculate(self.nominal_voltage(), 0.020, currents=[self.currentDraw])

    def output_current(self) -> wpimath.units.amperes:
        return self.currentDraw


class SparkMaxSim2175(SparkMaxSim):
    """
    A wrapper around rev's SparkMaxSim that automatically performs
    NetworkTables logging of sim-specific state.

    Also, the existing SparkMaxSim class seems to have a bug where the
    getMotorCurrent() method returns NaN. I do not know why. It would be good
    to figure this out. But for now we override it with our own custom logic.
    """

    def __init__(
        self,
        sparkMax: SparkMax,
        motor: DCMotor,
        *, nt: ntutil._NTFolder = ntutil._DummyNTFolder(),
    ) -> None:
        super().__init__(sparkMax, motor)
        self.encoder = self.getRelativeEncoderSim()
        self.motor = motor
        self.lastVbus: float = 0

        self.velocityTopic = nt.getFloatTopic("SimVelocity")
        self.currentTopic = nt.getFloatTopic("SimCurrent")

    def iterate(self, velocity, vbus, dt):
        super().iterate(velocity, vbus, dt)
        self.lastVbus = float(vbus)
        self.velocityTopic.set(self.encoder.getVelocity())
        self.currentTopic.set(self.getMotorCurrent())

    def getMotorCurrent(self) -> wpimath.units.amperes:
        speed: wpimath.units.radians_per_second = abs(self.encoder.getVelocity() / self.encoder.getVelocityConversionFactor()) * (2 * math.pi / 60)
        current = self.motor.current(speed, self.lastVbus * self.getAppliedOutput())
        return current
