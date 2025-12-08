from pyfrc.tests import *

import math
from wpimath.geometry import Rotation2d
import wpimath.units

import testutil
from utils import Vector2d


testutil.expecteq(Vector2d(1, 2) + Vector2d(3, 4), Vector2d(4, 6))
a = Vector2d(1, 2)
a += Vector2d(3, 4)
testutil.expecteq(a, Vector2d(4, 6))
testutil.notexpect(Vector2d(1, 2) + Vector2d(3, 4) == Vector2d(1, 2))

testutil.expecteq(Vector2d(1, 2) * 3, Vector2d(3, 6))
testutil.expecteq(-Vector2d(1, 2), Vector2d(-1, -2))

testutil.expecteq(Vector2d(1, 0).angle(), Rotation2d(0))
testutil.expecteq(Vector2d(0, 1).angle(), Rotation2d(math.pi / 2))
testutil.expecteq(Vector2d(-1, 0).angle(), Rotation2d(math.pi))
testutil.expecteq(Vector2d(0, -1).angle(), Rotation2d(3 * math.pi / 2))

testutil.expecteq(Vector2d(3, 0).normalized(), Vector2d(1, 0))
testutil.expecteq(Vector2d(0, 3).normalized(), Vector2d(0, 1))
