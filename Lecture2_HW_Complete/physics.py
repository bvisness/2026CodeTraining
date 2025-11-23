import math
import pyfrc.physics.core
from pyfrc.physics.core import PhysicsInterface
import rev
from wpilib import DriverStation, RobotController
from wpimath.system.plant import DCMotor
import wpimath.units

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
        nt = ntutil.folder("Sim")

        # Because moment of inertia scales linearly with mass, we can multiply
        # this later by the number of wheels being driven.
        compliantWheelMOI = momentOfInertiaForWheel(
            wpimath.units.inchesToMeters(4),
            wpimath.units.inchesToMeters(0.5),
            128 / 1000,
        )

        self.robot = robot
        self.armIntakeMotor = RotatingMotorObject(
            robot.arm.intakeMotor,
            DCMotor.NEO550(1).withReduction(6),
            momentOfInertia=compliantWheelMOI * 6, # six wheels on the intake
            nt=nt.folder("IntakeMotor")
        )

        self.vbusTopic = nt.getFloatTopic("VBus")

    def update_sim(self, now: float, tm_diff: float):
        vbus = RobotController.getBatteryVoltage()
        self.vbusTopic.set(vbus)

        self.armIntakeMotor.iterate(vbus, tm_diff)


class RotatingMotorObject:
    """
    A class used to simulate a rotating object driven by a SPARK MAX, e.g. a
    wheel or an arm.
    """

    def __init__(self,
                 spark: rev.SparkMax,
                 simMotor: DCMotor,
                 *,
                 momentOfInertia: wpimath.units.kilogram_square_meters,
                 viscousDragCoefficient: wpimath.units.newton_meters = 0.001,
                 friction: wpimath.units.newton_meters = 0.01,
                 nt: ntutil._NTFolder = ntutil._NTDummyFolder()):
        self.realSpark = spark
        self.simSpark = rev.SparkMaxSim(spark, simMotor)
        self.simMotor = simMotor
        self.relativeEncoder = self.simSpark.getRelativeEncoderSim()
        self.absoluteEncoder = self.simSpark.getAbsoluteEncoderSim()
        self.moi = momentOfInertia
        self.viscousDrag = viscousDragCoefficient
        self.friction = friction

        self.outputDutyCycleTopic = nt.getFloatTopic("OutputDutyCycle")
        self.outputCurrentTopic = nt.getFloatTopic("OutputCurrent")
        self.outputTorqueTopic = nt.getFloatTopic("OutputTorque")

        self.velocity = 0
        """rad/s"""

    def iterate(self, vbus: float, dt: float):
        # Compute output torque
        outputDutyCycle = self.simSpark.getAppliedOutput()
        isCoast = self.realSpark.configAccessor.getIdleMode() == rev.SparkBaseConfig.IdleMode.kCoast
        if outputDutyCycle == 0 and isCoast:
            outputTorque = 0
        else:
            motorVelocityRadPerS = self.relativeEncoder.getVelocity() / self.relativeEncoder.getVelocityConversionFactor() * (2 * math.pi / 60)
            outputCurrent = self.simMotor.current(motorVelocityRadPerS, outputDutyCycle * vbus)
            outputTorque = self.simMotor.torque(outputCurrent)

        self.outputDutyCycleTopic.set(outputDutyCycle)
        self.outputTorqueTopic.set(outputTorque)

        # Compute total torque (output plus drag / friction)
        viscousDragTorque = self.viscousDrag * -self.velocity
        frictionTorque = utils.sign_or_zero(-self.velocity) * self.friction # already Nm
        totalTorque = outputTorque + viscousDragTorque + frictionTorque

        # Torque -> accleration -> updated velocity
        angularAcceleration = totalTorque / self.moi # rad/(s^2)
        self.velocity += angularAcceleration * dt

        # Report velocity to SPARK sim in converted units
        vInConfiguredUnits = self.velocity / (2 * math.pi / 60) * self.relativeEncoder.getVelocityConversionFactor()
        self.simSpark.iterate(vInConfiguredUnits, vbus, dt)


def momentOfInertiaForWheel(
        outerDiameter: wpimath.units.meters,
        innerDiameter: wpimath.units.meters,
        mass: wpimath.units.kilograms,
        ) -> wpimath.units.kilogram_square_meters:
    """
    Computes the moment of inertia for a wheel.
    """
    r1 = innerDiameter / 2
    r2 = outerDiameter / 2
    return 0.5 * mass * (r2*r2 + r1*r1)
