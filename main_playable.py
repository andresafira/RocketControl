from constants import FREQUENCY, CAR_MAX_SPEED
from simulation import Simulation
import pygame
import sys


def main():
    """Performs a simulation controlled by the user, so the car dynamics can be
    tested and explored"""
    sim = Simulation(side='right', draw_Bounding_Box=True, draw_reference_line=False, draw_car_line=False)
    sim.dummy_simple_generator(num_dummy = 10, step = 700, side='right', speed = CAR_MAX_SPEED/3)
    run = True

    while run:
        pygame.time.Clock().tick(FREQUENCY)
        sim.update()

        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                run = False

        keys = pygame.key.get_pressed()

        if keys[pygame.K_w] or keys[pygame.K_UP]:
            sim.car.accelerate()
        if keys[pygame.K_a] or keys[pygame.K_LEFT]:
            sim.car.turn_left()
        if keys[pygame.K_d] or keys[pygame.K_RIGHT]:
            sim.car.turn_right()
        if keys[pygame.K_s] or keys[pygame.K_DOWN]:
            sim.car.brake()
        if keys[pygame.K_r]:
            sim.reset(side="right")
            sim.dummy_simple_generator(10, step = 700, side='right', speed = CAR_MAX_SPEED/3)

if __name__ == "__main__":
    main()
    sys.exit()
