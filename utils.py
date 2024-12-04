eps = 1e-4

def sgn(x: float|int) -> int:
    if x > eps:
        return 1
    elif x < -eps:
        return -1
    return 0
