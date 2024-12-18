from constants import *
from utils import Params
from simulation import Simulation
from control import FullPIDController, FullPDController
import pygame
import sys

def main(params: Params):
    """Performs a simulation controlled by the user,
    so the rocket dynamics can be tested and explored"""
    sim = Simulation(draw_reference_line=True, draw_rocket_line=True)
    windZ = 0
    windX = 0
    speed = FullPIDController(D_TIME, 0, MAX_THRUST)
    pos = FullPDController(D_TIME, pi/96)
    theta = FullPDController(D_TIME, MAX_NOZZLE_ANGLE)
    sim.rocket.set_controllers(speed_ctrl = speed, position_ctrl = pos, theta_ctrl = theta, speedCtrl = True)
    sim.rocket.set_control_params(params)
    sim.rocket.playable = False
    run = True

    while run:
        XR = WIDTH/2 + 40
        pygame.time.Clock().tick(FREQUENCY)
        sim.setWind(windX, windZ)
        sim.rocket.applyCommand(200, XR, windX, windZ)
        sim.add_reference_point(XR, sim.rocket.locZ)
        sim.update(verbose = False)

        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                run = False
        

        # keys = pygame.key.get_pressed()
        #
        # if keys[pygame.K_w] or keys[pygame.K_UP]:
        #     sim.rocket.increaseThrust()
        # if keys[pygame.K_a] or keys[pygame.K_LEFT]:
        #     sim.rocket.turnLeft()
        # if keys[pygame.K_d] or keys[pygame.K_RIGHT]:
        #     sim.rocket.turnRight()
        # if keys[pygame.K_s] or keys[pygame.K_DOWN]:
        #     sim.rocket.decreaseThrust()
        # if keys[pygame.K_r]:
        #     sim.reset()

if __name__ == "__main__":
    params = Params(xi_x = 0.98979, omega_x = 1.0764,
                    xi_theta = 0.430022, omega_theta = 6.4624,
                    xi_z = 0.8, omega_z = 1, k_z = 7)
    main(params)
    sys.exit()
