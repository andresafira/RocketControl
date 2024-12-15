from constants import *
from simulation import Simulation
from control import FullPIDController, PFController
import pygame
import sys

class SimpleControl:
    def __init__(self):
        pass

    def control(self, a = 0, b = 0, c = 0):
        return 0

def main():
    """Performs a simulation controlled by the user,
    so the rocket dynamics can be tested and explored"""
    sim = Simulation(draw_reference_line=False, draw_rocket_line=False)
    windZ = 0
    pos = SimpleControl()
    speed = FullPIDController(1, 1, 1, D_TIME, MAX_THRUST, 1)
    sim.rocket.set_controllers(speed_controller = speed, position_controller = pos)
    sim.rocket.playable = False
    run = True

    while run:
        pygame.time.Clock().tick(FREQUENCY)
        sim.setWind(0, windZ)
        sim.rocket.applyCommand(50, WIDTH/2, windZ)
        sim.update()

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
