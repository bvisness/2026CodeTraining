import math
import wpimath.units


# Several values in this file are sourced from data sheets for our actual robot
# hardware:
# - REV NEO data sheet: https://www.revrobotics.com/content/docs/REV-21-1650-DS.pdf
# - REV 3in MAXSwerve product page: https://www.revrobotics.com/rev-21-3005/

kWheelDiameter = wpimath.units.inchesToMeters(3)
"""Diameter of a drive wheel. Unit: m."""

kDriveMotorReduction = 4.71
""""High speed" gear ratio. Unit: ratio (N:1). (Source: REV MAXSwerve product page)"""

kSteerMotorReduction = 12
"""Gear ratio of the provided UltraPlanetary steering gearbox. (Source: REV MAXSwerve product page)"""

kDriveMotorFreeSpeed = 5676
"""Speed of a drive motor under no load. Unit: RPM. (Source: REV NEO data sheet)"""

kMaxSpeed = math.pi * kWheelDiameter * kDriveMotorFreeSpeed / 60.0 / kDriveMotorReduction
"""Maximum possible speed of a single drive wheel. Unit: m/s."""
