from pyfrc.tests import *
import wpimath.units

import testutil
from utils import Vector2d


testutil.expecteq(Vector2d(1, 2) + Vector2d(3, 4), Vector2d(4, 6))
a = Vector2d(1, 2)
a += Vector2d(3, 4)
testutil.expecteq(a, Vector2d(4, 6))
testutil.notexpect(Vector2d(1, 2) + Vector2d(3, 4) == Vector2d(1, 2))

testutil.expecteq(Vector2d(1, 2) * 3, Vector2d(3, 6))
