eps = 1e-4

def sgn(x: float|int) -> int:
    """Function that returns the sign of a number. Notice that there is a threshold of eps
    :param x: number to be analyzed
    :return: sign of x"""
    if x > eps:
        return 1
    elif x < -eps:
        return -1
    return 0

def clip(number, limit):
    """ Function that clips a number under a certain limit (works similarly to a mod function)
    :param number: number to be clipped
    :param limit: mod like number to be used as a clip bound
    :return: the clipped value
    """
    times = number // limit
    return number - times * limit
