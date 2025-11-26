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
        # Create two motors, one for the arm and one for the intake wheels on
        # the end of the arm.
        self.armMotor = rev.SparkMax(10, rev.SparkLowLevel.MotorType.kBrushless)
        self.intakeMotor = rev.SparkMax(11, rev.SparkLowLevel.MotorType.kBrushless)

        # HOMEWORK: Create the following variables here by getting objects from
        # the above motors:
        #  - self.armEncoder: get by calling getEncoder() on self.armMotor
        #  - self.armController: get by calling getClosedLoopController() on
        #    self.armMotor
        # You can use self.intakeEncoder as an example.
        self.intakeEncoder = self.intakeMotor.getEncoder()
        # More variables here...

        # Configure the two motors using configuration objects from configs.py.
        self.armMotor.configure(
            configs.armMotorConfig,
            rev.SparkMax.ResetMode.kResetSafeParameters,
            rev.SparkMax.PersistMode.kPersistParameters,
        )
        self.intakeMotor.configure(
            configs.intakeMotorConfig,
            rev.SparkMax.ResetMode.kResetSafeParameters,
            rev.SparkMax.PersistMode.kPersistParameters,
        )

        # The target angle we would like our arm to hit. This will be passed to
        # the setReference method of the SPARK MAX's closed-loop (PID)
        # controller in this subsystem's periodic method.
        self.desiredArmAngle = 0

        # Create two NetworkTables topics so we can view arm angles in
        # AdvantageScope.
        nt = ntutil.folder("Arm")
        self.angleDesiredTopic = ntutil.getFloatTopic("AngleDesired")
        self.angleActualTopic = ntutil.getFloatTopic("AngleActual")

        # Create Mechanism objects to view in AdvantageScope.
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

        # HOMEWORK: Call the setReference() method on the closed-loop
        # controller created in robotInit to tell the arm's SPARK MAX to target
        # an angle of self.desiredArmAngle. See 38:09 in the lecture video for
        # an example.

        # Code goes here...

        # HOMEWORK: Report values to AdvantageScope using
        # self.angleDesiredTopic and self.angleActualTopic, which were created
        # in __init__. You should send the following two values:
        #  - AngleDesired should be set to self.desiredArmAngle
        #  - AngleActual should be set to the arm encoder's current position.
        # See the first five minutes of the lecture video for an example.

        # Code goes here...

        # Update the mechanisms displayed in AdvantageScope. This will not work
        # unless you have defined armEncoder correctly in __init__!
        self.mechActual.update(armAngle=self.armEncoder.getPosition(),
                               wheelAngle=self.intakeEncoder.getPosition())
        self.mechDesired.update(armAngle=self.desiredArmAngle,
                                wheelAngle=0.4)

    def setDesiredArmAngle(self, angle: wpimath.units.radians):
        """
        Sets the desired angle for the arm, ensuring that it is within safe
        limits.
        """

        # HOMEWORK: The line below sets self.desiredArmAngle to the value of
        # the `angle` parameter, but it does not constrain the value to the
        # safety limits defined in constants.py. Modify it to make sure that
        # self.desiredArmAngle is always between constants.minArmAngle and
        # constants.maxArmAngle. The utils.clamp() function may be useful.
        self.desiredArmAngle = angle

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
