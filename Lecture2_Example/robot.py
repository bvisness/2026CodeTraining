import rev
import wpilib

import ntutil

# NetworkTables

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.steerMotor = rev.SparkMax(23, rev.SparkLowLevel.MotorType.kBrushless)
        self.steerEncoder = self.steerMotor.getAbsoluteEncoder()
        self.steerMotorPositionTopic = ntutil.getFloatTopic("SteerPosition")
        self.steerMotorVelocityTopic = ntutil.getFloatTopic("SteerVelocity")
    
    def robotPeriodic(self):
        self.steerMotorPositionTopic.set(self.steerEncoder.getPosition())
        self.steerMotorVelocityTopic.set(self.steerEncoder.getVelocity())
    
    def teleopPeriodic(self):
        # Control the motor here...
        pass
