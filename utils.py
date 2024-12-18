eps = 1e-4

class Params:
    def __init__(self, xi_x, omega_x,
                       xi_theta, omega_theta,
                       xi_z, omega_z, k_z):
        self.xi_x = xi_x
        self.omega_x = omega_x
        self.xi_theta = xi_theta
        self.omega_theta = omega_theta
        self.xi_z = xi_z
        self.omega_z = omega_z
        self.k_z = k_z

class Response:
    def __init__(self):
        self.theta = []
        self.theta_r = []
        self.x = []
        self.x_r = []
        self.z = []
        self.z_r = []
        self.thrust = []
        self.alpha = []

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
