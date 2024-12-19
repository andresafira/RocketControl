from constants import *
from math import fabs, sin, cos
from rocket import Rocket
from control import FullPIDController, FullPDController
from utils import Params, Response
import matplotlib.pyplot as plt
from cmaes import CMA
import numpy as np
import sys

# The following file is responsible to perform an optimization search to minimize
# the error of a controller, based on the parameters xi and omega of all the loops.
# It uses CMAes to do the search.
SPEED_CTRL = False # if True, the controller will control speed, else, it will control
                   # the vertical position instead.

def cost(actual: list, reference: list):
    if len(actual) != len(reference):
        raise Exception("Response and reference must have the same number of elements")
    s = 0
    for i in range(len(actual)):
        s += (actual[i] - reference[i])**2
    return s


def main(params: Params, plot: bool) -> Response:
    """Performs a simulation controlled by the user,
    so the rocket dynamics can be tested and explored"""
    windZ = 0
    windX = 0
    speed = FullPIDController(D_TIME, 0, MAX_THRUST)
    pos = FullPDController(D_TIME, pi/90)
    theta = FullPDController(D_TIME, MAX_NOZZLE_ANGLE)
    rocket = Rocket(locX = 0)
    rocket.set_controllers(speed_ctrl = speed, position_ctrl = pos,
                           theta_ctrl = theta, speedCtrl = SPEED_CTRL)
    rocket.set_control_params(params)
    rocket.playable = False

    resp = Response()
    
    max_time = 50
    XR = 40
    Z_input = 50
    
    for i in range(max_time*FREQUENCY):
        t = rocket.applyCommand(Z_input, XR, windX, windZ)
        rocket.move(windX, windZ)
        
        resp.theta_r.append(t)
        resp.alpha.append(rocket.nozzleAngle)
        resp.thrust.append(rocket.thrust)
        resp.x_r.append(XR)
        resp.x.append(rocket.locX)
        resp.theta.append(rocket.theta)
        if SPEED_CTRL:
            resp.z.append(rocket.speedZ)
        else:
            resp.z.append(rocket.locZ)
        resp.z_r.append(Z_input)

    if plot:
# Sample data (replace with actual data)
        t = [i*D_TIME for i in range(len(resp.x))]  # Time

        plt.figure(figsize=(6, 10))
# Plot 1: x_r and x vs t
        plt.subplot(3, 1, 1)
        plt.plot(t, resp.x_r, label="x_r", color="red")
        plt.plot(t, resp.x, label="x", color="blue")
        plt.ylabel("X Position (m)")
        plt.legend()
        plt.grid()

# Plot 2: theta_r and theta vs t
        plt.subplot(3, 1, 2)
        plt.plot(t, resp.theta_r, label="theta_r", color="red")
        plt.plot(t, resp.theta, label="theta", color="blue")
        plt.ylabel("Orientation (radians)")
        plt.legend()
        plt.grid()

# Plot 3: alpha vs t
        plt.subplot(3, 1, 3)
        plt.plot(t, resp.alpha, label="Alpha", color="black")
        plt.xlabel("Time (s)")
        plt.ylabel("Alpha (radians)")
        plt.grid()
# Adjust layout and show the plots
        plt.tight_layout()
        plt.savefig('output/X_opt.eps', format='eps')
        plt.show()

# Plot 1: z vs t
        plt.figure(figsize=(6, 10))
        label = "Z speed" if SPEED_CTRL else "Z position"
        plt.subplot(2, 1, 1)
        plt.plot(t, resp.z, label=label, color = "blue")
        plt.plot(t, resp.z_r, label="Reference" + label, color = 'red')
        plt.title(label)
        plt.ylabel("Z speed (m/s)" if SPEED_CTRL else "Z position (m)")
        plt.grid()

# Plot 2: thrust vs t
        plt.subplot(2, 1, 2)
        plt.plot(t, resp.thrust, label="Thrust", color="black")
        plt.xlabel("Time (s)")
        plt.ylabel("Thrust (N)")
        plt.grid()
# Adjust layout and show the plots
        plt.tight_layout()
        plt.savefig('output/Z_opt.eps', format='eps')
        plt.show()

    return resp

if __name__ == "__main__":
    # initial values (input) for the optimization algorithm
    initial = np.array([1, 10, 0.8, 10]) # only 4 params (only testing horizontal position)
    bounds = np.array([[0.1, 1], [1, 50], [0.1, 1], [1, 50]])
    opt = CMA(mean = initial, bounds = bounds, sigma = 1.3)
    for generation in range(100):
            solutions = []
            for _ in range(opt.population_size):
                x = opt.ask()
                params = Params(*x, 0.8, 1, 7)
                resp = main(params, plot = False)
                value = cost(resp.x + resp.z, resp.x_r + resp.z_r)
                solutions.append((x, value))
                print(f"#{generation} {value}")
            opt.tell(solutions)
    min = 1e10
    x_min = None
    for _ in range(opt.population_size):
        x = opt.ask()
        params = Params(*x, 0.8, 1, 7)
        resp = main(params, plot = False)
        value = cost(resp.x + resp.z, resp.x_r + resp.z_r)
        if value < min:
            min = value
            x_min = x
    print("\n\n\n\n\n\n", x_min)
    params = Params(*x_min, 0.8, 1, 7)
    main(params, plot = True)
    sys.exit()
