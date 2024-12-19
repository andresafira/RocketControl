from constants import *
from utils import Params, Response
from simulation import Simulation
from control import FullPIDController, FullPDController
import matplotlib.pyplot as plt
import pygame
import sys

# This file is responsible to perform the simulation of a specified controller
# The parameters are defined at the end of the file, and the 'main' function
# performs the simulation and also plots the graphs of it
# 

def main(params: Params) -> Response:
    """Performs a simulation controlled by the user,
    so the rocket dynamics can be tested and explored"""
    XR = WIDTH/2 + 20   # Horizontal Position of reference
    Z_input = 100       # Vertical Parameter of reference (it can be, speed or position, based on SPEED_CTRL)
    max_time = 50  # in seconds
    SPEED_CTRL = False # if True, the vertical controller will control the vertical speed
                       # if False, it will control the vertical position, instead.
    
    sim = Simulation(draw_reference_line=True, draw_rocket_line=True)
    windZ = 0
    windX = 0
    speed = FullPIDController(D_TIME, 0, MAX_THRUST)
    pos = FullPDController(D_TIME, pi/96)
    theta = FullPDController(D_TIME, MAX_NOZZLE_ANGLE)
    sim.rocket.set_controllers(speed_ctrl = speed, position_ctrl = pos, theta_ctrl = theta, speedCtrl = SPEED_CTRL)
    sim.rocket.set_control_params(params)
    resp = Response()
    sim.rocket.playable = False
    run = True
    time = 0

    while run:
        pygame.time.Clock().tick(FREQUENCY)
        time += D_TIME
        sim.setWind(windX, windZ)
        t = sim.rocket.applyCommand(Z_input, XR, windX, windZ)
        sim.add_reference_point(XR, sim.rocket.locZ)
        sim.update(verbose = False)
        
        resp.theta_r.append(t)
        resp.alpha.append(sim.rocket.nozzleAngle)
        resp.thrust.append(sim.rocket.thrust)
        resp.x_r.append(XR)
        resp.x.append(sim.rocket.locX)
        resp.theta.append(sim.rocket.theta)
        if SPEED_CTRL:
            resp.z.append(sim.rocket.speedZ)
        else:
            resp.z.append(sim.rocket.locZ)
        resp.z_r.append(Z_input)

        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                run = False
        
        if time > max_time:
            run = False

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
    plt.savefig('output/x.eps', format='eps')
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
    plt.savefig('output/z.eps', format='eps')
    plt.show()

    return resp


if __name__ == "__main__":
    # The meaning of the following parameters is specified at the report pdf file
    params = Params(xi_x = 1, omega_x = 10,
                    xi_theta = 0.8, omega_theta = 10,
                    xi_z = 0.8, omega_z = 1, k_z = 7)
    main(params)
    sys.exit()
