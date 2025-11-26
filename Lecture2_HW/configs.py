import math
import rev


# =============================================================================
# This file contains configuration objects that are used for SPARK MAX
# controllers.
# =============================================================================

# HOMEWORK: Update the P, I, and D constants below to make the simulated arm
# move quickly but accurately. Start by tuning P until the arm moves quickly
# but oscillates around the desired angle. Then increase D until the
# oscillation is under control. For this exercise you will not need to adjust
# I, but you may experiment with it if you wish.
armMotorConfig = rev.SparkMaxConfig()
armMotorConfig.encoder.positionConversionFactor(math.pi * 2) # rotations -> radians
armMotorConfig.encoder.velocityConversionFactor(math.pi * 2 / 60) # RPM -> rad/s
armMotorConfig.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
armMotorConfig.closedLoop.pid(
    p=0.05,  # Units: output/rad (e.g. 0.1 => 0.1 motor output per 1 radian of error)
    i=0,     # Units: output/(rad*s) (e.g. 0.1 => 0.1 motor output per 1 second at 1 radian of error)
    d=0,     # Units: output/(rad/s) (e.g. 0.1 => 0.1 motor output when approaching setpoint at 1 rad/s)
)

intakeMotorConfig = rev.SparkMaxConfig()
intakeMotorConfig.encoder.positionConversionFactor(math.pi * 2) # rotations -> radians
intakeMotorConfig.encoder.velocityConversionFactor(math.pi * 2 / 60) # RPM -> rad/s
