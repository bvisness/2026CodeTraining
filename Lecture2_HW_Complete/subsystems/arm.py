import rev

import constants
import ntutil
import utils


class Arm():
    def __init__(self):
        self.angleMotor = rev.SparkMax(10, rev.SparkLowLevel.MotorType.kBrushless)
        self.intakeMotor = rev.SparkMax(11, rev.SparkLowLevel.MotorType.kBrushless)

        self.intakeEncoder = self.intakeMotor.getEncoder()

        self.desiredAngle = 0

        nt = ntutil.folder("Arm")
        self.desiredAngleTopic = nt.getFloatTopic("AngleDesired")
        self.intakeDesiredSpeed = nt.getFloatTopic("IntakeSpeedDesired")
        self.intakeActualSpeed = nt.getFloatTopic("IntakeSpeedActual")

    def periodic(self):
        self.desiredAngleTopic.set(self.desiredAngle)
        self.intakeDesiredSpeed.set(self.intakeMotor.get())
        self.intakeActualSpeed.set(self.intakeEncoder.getVelocity())

    def setDesiredAngle(self, angleRadians: float):
        safeAngle = utils.clamp(angleRadians, constants.kArmMinAngle, constants.kArmMaxAngle)
        self.desiredAngle = safeAngle
