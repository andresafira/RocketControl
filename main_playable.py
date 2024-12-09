from constants import FREQUENCY
from simulation import Simulation
import pygame
import sys


def main():
    """Performs a simulation controlled by the user, so the car dynamics can be
    tested and explored"""
    sim = Simulation(draw_reference_line=False, draw_rocket_line=False)
    run = True

    while run:
        pygame.time.Clock().tick(FREQUENCY)
        sim.update()

        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                run = False

        keys = pygame.key.get_pressed()

        if keys[pygame.K_w] or keys[pygame.K_UP]:
            sim.rocket.increaseThrust()
        if keys[pygame.K_a] or keys[pygame.K_LEFT]:
            sim.rocket.turnLeft()
        if keys[pygame.K_d] or keys[pygame.K_RIGHT]:
            sim.rocket.turnRight()
        if keys[pygame.K_s] or keys[pygame.K_DOWN]:
            sim.rocket.decreaseThrust()
        if keys[pygame.K_r]:
            sim.reset()

if __name__ == "__main__":
    main()
    sys.exit()
