from utils import sgn, eps
from constants import D_THRUST, MAX_THRUST, D_NOZZLE_ANGLE, MAX_NOZZLE_ANGLE, POS_CM, POS_CG
from constants import AIR_RES_X, AIR_RES_Z, ROCKET_MASS, GRAVITY, INERTIA, D_TIME, INERTIA_NOZZLE
from math import cos, sin, pi, fabs, sqrt, tan
from control import PFController, FullPIDController


def windForce(k, w, v):
    return 0 #k*(w-v)


class Rocket:
    """Rocket implementation class"""
    def __init__(self, locX: float = 0, locZ: float = 0, theta: float = 0,
                       speedX: float = 0, speedZ: float = 0, omega: float = 0):
        """Initialization method. It generates a car in a
            given position with a given initial speed.
            If DUMMY is passed as True, then the car will only
            move with the given constant speed"""

        self.locX: float = locX
        self.locZ: float = locZ
        self.theta: float = theta

        self.speedX: float = speedX
        self.speedZ: float = speedZ
        self.omega: float = omega

        self.thrust: float = 0
        
        # It tells if the car will be controlled by the user
        # or by the specified control system
        self.playable: bool = True
        self.nozzleAngle: float = 0

    def set_controllers(self, speed_controller: PFController,
                              position_controller: FullPIDController):
        self.speed_controller = speed_controller
        self.position_controller = position_controller

    def increaseThrust(self):
        # Only accessible if self.playable = True
        self.thrust += D_THRUST
        self.thrust = min(self.thrust, MAX_THRUST)

    def decreaseThrust(self):
        # Only accessible if self.playable = True
        self.thrust -= D_THRUST
        self.thrust = max(self.thrust, 0)

    def turnRight(self):
        # Only accessible if self.playable = True
        self.nozzleAngle -= D_NOZZLE_ANGLE
        self.nozzleAngle = max(self.nozzleAngle, -MAX_NOZZLE_ANGLE)

    def turnLeft(self):
        # Only accessible if self.playable = True
        self.nozzleAngle += D_NOZZLE_ANGLE
        self.nozzleAngle = min(self.nozzleAngle, MAX_NOZZLE_ANGLE)

    def apply_command(self, vr, zr):
        """Method that applies the gets the command of the controllers and
        apply to the vehicle motion"""
        pass

    def move(self, windX, windZ):
        print(f'X = {self.locX:.2f}, Z = {self.locZ:.2f}, theta = {self.theta:.2f}, nozzle = {self.nozzleAngle:.2f}, T = {self.thrust}, sx = {self.speedX:.2f}, sz = {self.speedZ:.2f}, omega = {self.omega:.2f}')
        self.locX += self.speedX * D_TIME
        self.locZ += self.speedZ * D_TIME
        self.theta += self.omega * D_TIME
        
        windForceX: float = windForce(AIR_RES_X, windX, self.speedX)
        windForceZ: float = windForce(AIR_RES_Z, windZ, self.speedZ)

        forceX: float = windForceX + self.thrust * sin(self.theta + self.nozzleAngle)
        forceZ: float = windForceZ + self.thrust * cos(self.theta + self.nozzleAngle) - ROCKET_MASS*GRAVITY
        torque: float = (POS_CG - POS_CM) * (windForceX * cos(self.theta) - windForceZ * sin(self.theta)) - self.thrust * POS_CM * sin(self.nozzleAngle)

        self.speedX += forceX / ROCKET_MASS * D_TIME
        self.speedZ += forceZ / ROCKET_MASS * D_TIME
        self.omega += torque / INERTIA * D_TIME
        
        if self.playable:
            # The steering wheel inertia is only applied under user control in order to
            # give a better user experience. The control system do not experience such
            # effect since it is supposed that it has full control of the steering wheel
            self.nozzleAngle -= sgn(self.nozzleAngle) * INERTIA_NOZZLE
        
        if fabs(self.nozzleAngle) < 2 * eps * pi:
            self.nozzleAngle = 0
        if fabs(self.speedX) < eps:
            self.speedX = 0
        if fabs(self.speedZ) < eps:
            self.speedZ = 0
