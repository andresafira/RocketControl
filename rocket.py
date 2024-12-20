from utils import sgn, eps, Params
from constants import D_THRUST, MAX_THRUST, D_NOZZLE_ANGLE, MAX_NOZZLE_ANGLE, POS_CM, POS_CG
from constants import AIR_RES_X, AIR_RES_Z, ROCKET_MASS, GRAVITY, INERTIA, D_TIME, INERTIA_NOZZLE
from constants import THRUST_THRESHOLD
from math import cos, sin, pi, fabs, sqrt, tan
from control import FullPIDController, FullPDController


def windForce(k, w, v):
    return k*(w-v)*fabs(w-v)


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

    def set_controllers(self, speed_ctrl: FullPIDController,
                        position_ctrl: FullPDController,
                        theta_ctrl = FullPDController,
                        speedCtrl: bool = True):
        self.speed_controller = speed_ctrl
        self.position_controller = position_ctrl
        self.theta_controller = theta_ctrl
        self.speedCtrl = speedCtrl

    def set_control_params(self, params):
        self.xi_x = params.xi_x
        self.omega_x = params.omega_x
        self.xi_theta = params.xi_theta
        self.omega_theta = params.omega_theta
        self.xi_z = params.xi_z
        self.omega_z = params.omega_z
        self.k_z = params.k_z

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

    def updateSpeedController(self, windZ):
        sign = sgn(windZ - self.speedZ)
        kp = 2 * ROCKET_MASS * self.xi_z * self.omega_z + 2 * AIR_RES_Z * windZ
        kd = 0
        ki = ROCKET_MASS * self.omega_z ** 2
        wind_term = AIR_RES_Z * (windZ**2 + self.speedZ**2) * sign
        offset_command = ROCKET_MASS * GRAVITY - wind_term 
        self.speed_controller.update_constants(kp, ki, kd, offset_command)

    def updateVerticalController(self, windZ):
        sign = sgn(windZ - self.speedZ)
        kp = ROCKET_MASS * self.omega_z**2 * (1+ 2 * self.xi_z * self.k_z)
        kd = ROCKET_MASS * self.omega_z * (self.k_z + 2*self.xi_z) - 2*AIR_RES_Z*sign*windZ
        ki = ROCKET_MASS * self.k_z * self.omega_z ** 3
        wind_term = AIR_RES_Z * (windZ**2 + self.speedZ**2) * sign
        offset_command = ROCKET_MASS * GRAVITY - wind_term 
        self.speed_controller.update_constants(kp, ki, kd, offset_command)

    def updatePositionController(self, windX, windZ):
        sign_x = sgn(windX - self.speedX)
        sign_z = sgn(windZ - self.speedZ)
        T = max(THRUST_THRESHOLD, self.thrust)
        kd = 2 * (ROCKET_MASS * self.omega_x * self.xi_x - sign_x * AIR_RES_X * windX) / T
        kp = ROCKET_MASS * self.omega_x**2 / T
        offset_command = -sign_x * AIR_RES_X * (windX**2 + self.speedX**2) / T - self.nozzleAngle
        self.position_controller.update_constants(kp, kd, offset_command)

        beta = (POS_CG - POS_CM) * AIR_RES_X * (windX - self.speedX)**2 * sign_x
        gamma = (POS_CG - POS_CM) * AIR_RES_Z * (windZ - self.speedZ)**2 * sign_z
        kd = -2 * INERTIA * self.xi_theta * self.omega_theta / (T * POS_CM)
        kp = (gamma - INERTIA * self.omega_theta**2) / (T * POS_CM)
        offset_command = beta / (T * POS_CM)
        self.theta_controller.update_constants(kp, kd, offset_command)
    
    def applyCommand(self, vz, xr, windX, windZ):
        """Method that applies the gets the command of the controllers and
        apply to the vehicle motion"""
        self.updatePositionController(windX, windZ)
        theta_r = self.position_controller.control(xr, self.locX)
        self.nozzleAngle = self.theta_controller.control(theta_r, self.theta)
        
        if self.speedCtrl:
            self.updateSpeedController(windZ)
            self.thrust = self.speed_controller.control(vz, self.speedZ)
        else:
            self.updateVerticalController(windZ)
            self.thrust = self.speed_controller.control(vz, self.locZ)
        return theta_r

    def move(self, windX, windZ, verbose = False):
        if verbose:
            print(f'X = {self.locX:.2f}, Z = {self.locZ:.2f}, theta = {self.theta*180/pi:.2f}, nozzle = {self.nozzleAngle*180/pi:.2f}, T = {self.thrust:.2f}, sx = {self.speedX:.2f}, sz = {self.speedZ:.2f}, omega = {self.omega*pi/180:.2f}')
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
