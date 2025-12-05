import wpimath.units

import ntutil
import utils


class RotatingObject:
    """
    A class used to simulate the physics of a rotating object with inertia,
    drag, and friction.
    """

    def __init__(
        self,
        *, momentOfInertia: wpimath.units.kilogram_square_meters,
        viscousDragCoefficient = 0.001,
        friction: wpimath.units.newton_meters = 0.01,
        nt: ntutil._NTFolder = ntutil._DummyNTFolder()
    ):
        self.moi = momentOfInertia
        self.viscousDrag = viscousDragCoefficient
        self.friction = friction

        self.velocity: wpimath.units.radians_per_second = 0

        self.inputTorqueTopic = nt.getFloatTopic("InputTorque")
        self.viscousTorqueTopic = nt.getFloatTopic("ViscousDragTorque")
        self.frictionTorqueTopic = nt.getFloatTopic("FrictionTorque")
        self.totalTorqueTopic = nt.getFloatTopic("TotalTorque")
        self.velocityTopic = nt.getFloatTopic("Velocity")

    def iterate(self, inputTorque: wpimath.units.newton_meters, dt: float):
        # Compute total torque (output plus drag / friction)
        viscousDragTorque = self.viscousDrag * -self.velocity
        frictionTorque = utils.sign_or_zero(-self.velocity) * self.friction # already Nm
        totalTorque = inputTorque + viscousDragTorque + frictionTorque

        # Since friction is a constant force, naively applying it at low speeds
        # can cause us to reverse direction. We stomp on this whole situation
        # by clamping the total torque so that we never reverse direction in a
        # single tick.
        #
        # TODO: This is still sort of wrong. We're not going to land exactly on
        # zero, but then we will ignore legitimate torques close to zero.
        torqueLimit = self.moi * -self.velocity / dt
        if totalTorque < torqueLimit < 0 or 0 < torqueLimit < totalTorque:
            totalTorque = torqueLimit

        # Torque -> accleration -> updated velocity
        angularAcceleration = totalTorque / self.moi # rad/(s^2)
        self.velocity += angularAcceleration * dt

        self.inputTorqueTopic.set(inputTorque)
        self.viscousTorqueTopic.set(viscousDragTorque)
        self.frictionTorqueTopic.set(frictionTorque)
        self.totalTorqueTopic.set(totalTorque)
        self.velocityTopic.set(self.velocity)
    
    def getVelocity(self) -> wpimath.units.radians_per_second:
        return self.velocity


def moiForWheel(
    mass: wpimath.units.kilograms,
    outerDiameter: wpimath.units.meters,
    innerDiameter: wpimath.units.meters,
) -> wpimath.units.kilogram_square_meters:
    """
    Computes the moment of inertia for a wheel.
    """
    r1 = innerDiameter / 2
    r2 = outerDiameter / 2
    return 0.5 * mass * (r2*r2 + r1*r1)


def moiForArm(
    mass: wpimath.units.kilograms,
    centerOfMassRadius: wpimath.units.meters,
) -> wpimath.units.kilogram_square_meters:
    """
    Computes the moment of inertia for an arm or other point mass.
    """
    return mass * centerOfMassRadius * centerOfMassRadius
