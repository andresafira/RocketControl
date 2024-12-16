from constants import *
from simulation import Simulation
from control import FullPIDController, FullPDController
import pygame
import sys

class SimpleControl:
    def __init__(self):
        pass

    def update_constants(self, a=0, b=0, c=0, d=0, e=0):
        pass

    def control(self, a=0, b=0, c=0, d=0, e=0):
        return 5 * ROCKET_MASS * GRAVITY

def main():
    """Performs a simulation controlled by the user,
    so the rocket dynamics can be tested and explored"""
    sim = Simulation(draw_reference_line=False, draw_rocket_line=False)
    windZ = 0
    windX = 0
    speed = FullPIDController(D_TIME, MAX_THRUST)
    # speed = SimpleControl()
    pos = FullPDController(D_TIME, pi/96)
    theta = FullPDController(D_TIME, pi/2)
    sim.rocket.set_controllers(speed_ctrl = speed, position_ctrl = pos, theta_ctrl = theta)
    sim.rocket.playable = False
    run = True

    while run:
        pygame.time.Clock().tick(FREQUENCY)
        sim.setWind(0, windZ)
        sim.rocket.applyCommand(50, WIDTH/2+10, windX, windZ)
        sim.update(verbose=True)

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
    main()
    sys.exit()
