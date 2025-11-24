import rev
import wpilib
from wpilib import SmartDashboard
import wpimath.units

import configs
import constants
import ntutil
import utils


# =============================================================================
# This file contains the Arm subsystem, which demonstrates how to use PID to
# drive a mechanism to a specified position. You can see a visualization of the
# arm's state in AdvantageScope.
# =============================================================================

class Arm():
    def __init__(self):
        # HOMEWORK: Create two SparkMax variables here to control two motors:
        #  - self.armMotor (with ID 10)
        #  - self.intakeMotor (with ID 11)

        # Code goes here...

        # HOMEWORK: Do NOT change these lines. These should help you verify
        # that you created the motors correctly above.
        self.armEncoder = self.armMotor.getEncoder()
        self.intakeEncoder = self.intakeMotor.getEncoder()
        self.armController = self.armMotor.getClosedLoopController()

        # HOMEWORK: Configure the motors using config objects from configs.py.
        #  - self.armMotor should be configured with configs.armMotorConfig.
        #  - self.intakeMotor should be configured with
        #    configs.intakeMotorConfig.
        # In both cases, you can use the following two values for the
        # resetMode and persistMode parameters:
        #  - resetMode: rev.SparkMax.ResetMode.kResetSafeParameters
        #  - persistMode: rev.SparkMax.PersistMode.kPersistParameters

        # Code goes here...

        # The target angle we would like our arm to hit. This will be passed to
        # the setReference method of the SPARK MAX's closed-loop (PID)
        # controller in this subsystem's periodic method.
        #
        # HOMEWORK: Read the above comment, and do not touch this field :)
        self.desiredArmAngle = 0

        # HOMEWORK: Create two NetworkTables topics so we can view arm angles
        # in AdvantageScope. You should have the following two float topics:
        #  - AngleDesired
        #  - AngleActual
        nt = ntutil.folder("Arm")
        # Create topics here...

        # Create Mechanism objects to view in AdvantageScope. You should not
        # touch these fields.
        self.mechActual = self.Mechanism(nt.topicName("MechanismActual"),
                                         armColor=wpilib.Color.kRed,
                                         wheelColor=wpilib.Color.kGreen)
        self.mechDesired = self.Mechanism(nt.topicName("MechanismDesired"),
                                          armColor=wpilib.Color.kGray,
                                          wheelColor=wpilib.Color.kGray)

    def periodic(self):
        """
        Runs subsystem logic on every tick. Should be called from robotPeriodic.
        """

        # HOMEWORK: Set the reference value for the arm's PID controller to
        # self.desiredArmAngle. Remember to use position mode instead of
        # velocity mode!

        # Code goes here...

        # HOMEWORK: Report values to AdvantageScope using the NetworkTables
        # topics created in __init__. You should send the following two values:
        #  - AngleDesired should be set to self.desiredArmAngle
        #  - AngleActual should be set to the arm encoder's current position.

        # Code goes here...

        # Update the mechanisms displayed in AdvantageScope. You do not need to
        # modify these lines, although you can read them if perhaps you need a
        # hint :)
        self.mechActual.update(armAngle=self.armEncoder.getPosition(),
                               wheelAngle=self.intakeEncoder.getPosition())
        self.mechDesired.update(armAngle=self.desiredArmAngle,
                                wheelAngle=0.4)

    def setDesiredArmAngle(self, angle: wpimath.units.radians):
        """
        Sets the desired angle for the arm, ensuring that it is within safe
        limits.
        """

        # HOMEWORK: Set self.desiredArmAngle to the `angle` parameter above,
        # but make sure that the value is restricted to fall within
        # constants.armMinAngle and constants.armMaxAngle. (You can see the
        # definitions of these values in constants.py.)

        # Code goes here...

    def setIntakeSpeed(self, speed: float):
        """
        Sets the intake wheels to an arbitrary speed.
        """
        self.intakeMotor.set(speed)


    # =========================================================================
    # The code below this point is used for visualization and should not be
    # modified as part of the homework.

    class Mechanism:
        """
        A utility class that creates a Mechanism2d for display in
        AdvantageScope. This makes it easy to display both the desired and
        actual mechanism states.
        """
        def __init__(self, ntName: str, armColor: wpilib.Color, wheelColor: wpilib.Color):
            canvasWidth = 1 # m
            canvasHeight = 1 # m
            self.mech = wpilib.Mechanism2d(width=canvasWidth, height=canvasHeight)
            self.root = self.mech.getRoot("ArmBase",
                                          x=wpimath.units.inchesToMeters(4),
                                          y=wpimath.units.inchesToMeters(4))
            self.arm = self.root.appendLigament("Arm",
                                                length=wpimath.units.inchesToMeters(24),
                                                angle=0,
                                                color=wpilib.Color8Bit(armColor))
            self.intake = self.arm.appendLigament("Intake",
                                                  length=wpimath.units.inchesToMeters(2),
                                                  angle=0,
                                                  color=wpilib.Color8Bit(wheelColor))
            
            # Draw a "wheel" in the dumbest way possible
            i1 = self.intake.appendLigament("I1",
                                            length=wpimath.units.inchesToMeters(2),
                                            angle=90,
                                            color=wpilib.Color8Bit(wheelColor))
            i2 = i1.appendLigament("I2",
                                   length=wpimath.units.inchesToMeters(4),
                                   angle=90,
                                   color=wpilib.Color8Bit(wheelColor))
            i3 = i2.appendLigament("I3",
                                   length=wpimath.units.inchesToMeters(4),
                                   angle=90,
                                   color=wpilib.Color8Bit(wheelColor))
            i4 = i3.appendLigament("I4",
                                   length=wpimath.units.inchesToMeters(4),
                                   angle=90,
                                   color=wpilib.Color8Bit(wheelColor))
            i5 = i4.appendLigament("I5",
                                   length=wpimath.units.inchesToMeters(2),
                                   angle=90,
                                   color=wpilib.Color8Bit(wheelColor))
            SmartDashboard.putData(ntName, self.mech)

        def update(self, armAngle: wpimath.units.radians, wheelAngle: wpimath.units.radians):
            self.arm.setAngle(wpimath.units.radiansToDegrees(armAngle))
            self.intake.setAngle(wpimath.units.radiansToDegrees(-wheelAngle))
