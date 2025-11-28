import math
import random
import typing

import pyfrc.physics.core
from pyfrc.physics.core import PhysicsInterface
from rev import SparkMaxSim
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

        # Update simulated electrical state
        currents = [self.drivetrain.get_current_draw()]
        self.battery.update_sim(sum(currents), tm_diff)
        RoboRioSim.setVInVoltage(self.battery.output_voltage())
        RoboRioSim.setVInCurrent(self.battery.output_current())
        self.batteryVoltageTopic.set(RobotController.getBatteryVoltage())
        self.batteryCurrentTopic.set(RobotController.getInputCurrent())


class SwerveModuleSim:
    def __init__(self, module: SwerveModule):
        self.driveMotorSim = DCMotor.NEO(1).withReduction(constants.kDriveMotorReduction)
        self.steerMotorSim = DCMotor.NEO550(1).withReduction(constants.kSteerMotorReduction)
        self.driveSparkSim = SparkMaxSim(module.driveMotor, self.driveMotorSim)
        self.steerSparkSim = SparkMaxSim(module.steerMotor, self.steerMotorSim)
        self.driveRelativeEncoderSim = self.driveSparkSim.getRelativeEncoderSim()
        self.steerRelativeEncoderSim = self.steerSparkSim.getRelativeEncoderSim()
        self.steerAbsoluteEncoderSim = self.steerSparkSim.getAbsoluteEncoderSim()
        self.angleOffset = module.angleOffset

        # Calculates the velocity of the drive motor, roughly simulating
        # inertia and friction. Note that this limits the SPEED of the motors.
        # Linear change to velocity means constant acceleration, so the units
        # here check out.
        # TODO: Delete this and use an actual physics model.
        self.driveSpeedLimiter = SlewRateLimiter(9.8 * 1) # 1 G (m/s^2)

        # Use a PID controller to simulate velocity of the steer motor
        # TODO: Delete this and use an actual physics model.
        self.steerController = PIDController(100, 0, 0) # magic numbers!
        self.steerController.enableContinuousInput(-math.pi, math.pi)

        # Randomize the starting rotation to simulate what we have when we take
        # the field
        self.steerSparkSim.setPosition(random.uniform(-math.pi, math.pi))

        # Cached values from iterate()
        self.driveCurrent: wpimath.units.amperes = 0
        self.steerCurrent: wpimath.units.amperes = 0

    def update_sim(self, vbus: float, tm_diff: float):
        targetSpeed = self.driveSparkSim.getSetpoint() if DriverStation.isEnabled() else 0
        driveSpeed = self.driveSpeedLimiter.calculate(targetSpeed)
        self.driveSparkSim.iterate(driveSpeed, vbus, tm_diff)

        # TODO: redo all of this, it is Jank and Wrong
        targetAngle = self.steerSparkSim.getSetpoint()
        currentAngle = wpimath.angleModulus(self.steerSparkSim.getPosition())
        self.steerController.setSetpoint(targetAngle)
        steerSpeed = self.steerController.calculate(currentAngle)
        self.steerSparkSim.iterate(1, vbus, tm_diff) # TODO: 1 mystery unit per second?
        self.steerAbsoluteEncoderSim.iterate(1, tm_diff) # TODO: Translate from steer motor velocity into encoder angle (should be straightforward gearing)

        # TODO: Collect this little song and dance into a utility function
        driveVelocity: wpimath.units.radians_per_second = self.driveRelativeEncoderSim.getVelocity() / self.driveRelativeEncoderSim.getVelocityConversionFactor() * (2 * math.pi / 60)
        steerVelocity: wpimath.units.radians_per_second = self.steerRelativeEncoderSim.getVelocity() / self.steerRelativeEncoderSim.getVelocityConversionFactor() * (2 * math.pi / 60)
        self.driveCurrent = self.driveMotorSim.current(driveVelocity, vbus * self.driveSparkSim.getAppliedOutput())
        self.steerCurrent = self.steerMotorSim.current(driveVelocity, vbus * self.steerSparkSim.getAppliedOutput())

    def get_state(self) -> SwerveModuleState:
        return SwerveModuleState(
            wpimath.units.meters_per_second(self.driveSparkSim.getVelocity()),
            Rotation2d(self.steerAbsoluteEncoderSim.getPosition() - self.angleOffset)
        )

    def get_current_draw(self) -> wpimath.units.amperes:
        # TODO: y u no output current? u move motor? anything work?
        return self.driveCurrent + self.steerCurrent


class SwerveDriveSim:
    def __init__(self, frontLeft: SwerveModuleSim, frontRight: SwerveModuleSim, backLeft: SwerveModuleSim, backRight: SwerveModuleSim):
        wd = wpimath.units.inchesToMeters(12.5) # wheel distance
        self.kinematics = SwerveDrive4Kinematics(
            Translation2d(wd, wd),
            Translation2d(wd, -wd),
            Translation2d(-wd, wd),
            Translation2d(-wd, -wd),
        )
        self.modules: tuple[SwerveModuleSim, SwerveModuleSim, SwerveModuleSim, SwerveModuleSim] = (frontLeft, frontRight, backLeft, backRight)

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
        return sum(s.get_current_draw() for s in self.modules)


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
