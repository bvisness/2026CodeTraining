from typing import Type


def expect(val):
    if not val:
        raise RuntimeError(f"expected truthy value, but got {val}")

def notexpect(val):
    if val:
        raise RuntimeError(f"expected falsy value, but got {val}")

def expecteq(actual, expected):
    if actual != expected:
        raise RuntimeError(f"expected {expected} but got {actual}")

def expecterror(f, error: Type[BaseException] = Exception):
    try:
        f()
    except error:
        return
    except Exception as e:
        raise e
    raise RuntimeError(f"expected function to raise {error.__name__}, but no exception was thrown")
