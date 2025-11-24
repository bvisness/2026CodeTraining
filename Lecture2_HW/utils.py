# =============================================================================
# This file contains simple utilities that are generally useful throughout all
# our Python code.
# =============================================================================

def lerp(a: float, b: float, t: float) -> float:
    """
    "Lerp" is short for "Linear intERPolation". It can be used to interpolate
    (blend) between two values, `a` and `b`, by a factor of `t`. Examples:

        lerp(10, 20, t=0)   => 10
        lerp(10, 20, t=1)   => 20
        lerp(10, 20, t=0.5) => 15  # halfway between a and b

    """
    return (1-t) * a + t * b

def clamp(value: float, min: float, max: float) -> float:
    """
    Limits the input `value` to the given `min` and `max`, useful for enforcing
    a safe range of values. Examples:

        clamp(-10, min=-1, max=1) => -1   # limited to min
        clamp(10, min=-1, max=1)  => 1    # limited to max
        clamp(0.5, min=-1, max=1) => 0.5  # unchanged

    """
    assert min <= max, f"in utils.clamp: min ({min}) and max ({max}) are reversed"
    if value < min:
        return min
    elif value > max:
        return max
    else:
        return value

def remap(value: float, before: tuple[float, float], after: tuple[float, float]) -> float:
    """
    Takes two ranges of values, `before` and `after`, and maps the input
    `value` from `before` to `after`. This is often useful for joystick inputs,
    where for example you may want -1 and 1 to correspond to your minimum and
    maximum robot speeds. Examples:

        remap(  1, (-1, 1), (-math.pi, math.pi)) => math.pi
        remap( -1, (-1, 1), (-math.pi, math.pi)) => -math.pi
        remap(0.5, (-1, 1), (-math.pi, math.pi)) => math.pi/2

    """
    t = (value - before[0]) / (before[1] - before[0])
    return lerp(after[0], after[1], t)

def sign(v: float) -> float:
    """
    Extracts the sign of a number: -1 for negative numbers, 1 for positive
    numbers and zero.
    """
    if v < 0:
        return -1
    else:
        return 1

def sign_or_zero(v: float) -> float:
    """
    Extracts the sign of a number: -1 for negative numbers, 1 for positive
    numbers, and 0 for zero.
    """
    if v == 0:
        return 0
    else:
        return sign(v)
