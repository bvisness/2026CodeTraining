import math
import rev


armMotorConfig = rev.SparkMaxConfig()
armMotorConfig.encoder.positionConversionFactor(math.pi * 2) # rotations -> radians
armMotorConfig.encoder.velocityConversionFactor(math.pi * 2 / 60) # RPM -> rad/s
armMotorConfig.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
armMotorConfig.closedLoop.pid(
    p=10,   # Units: speed/rad (e.g. 0.1 => 0.1 speed per 1 radian of error)
    i=0,    # Units: speed/(rad*s) (e.g. 0.1 => 0.1 speed per 1 second at 1 radian of error)
    d=0.8,  # Units: speed/(rad/s) (e.g. 0.1 => 0.1 speed when approaching setpoint at 1 rad/s)
)

intakeMotorConfig = rev.SparkMaxConfig()
intakeMotorConfig.encoder.positionConversionFactor(math.pi * 2) # rotations -> radians
intakeMotorConfig.encoder.velocityConversionFactor(math.pi * 2 / 60) # RPM -> rad/s
