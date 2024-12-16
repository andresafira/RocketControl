from constants import D_TIME, BLACK, WHITE, RED, M2P, WIDTH, HEIGHT
from constants import ROCKET_WIDTH, ROCKET_HEIGHT, FIRE_WIDTH, FIRE_HEIGHT
from constants import ROCKET_SPRITE, BACKGROUND_SPRITE, FIRE_SPRITE, MAX_THRUST
from rocket import Rocket

from utils import clip

import pygame
from pygame.transform import scale, rotate, flip
from pygame.image import load
from pygame.locals import *

from math import pi, sin, cos, fabs
from rocket import Rocket


class Simulation:
    """Main simulation engine that manages the interaction between objects"""
    def __init__(self, draw_reference_line: bool = False, draw_rocket_line: bool = False):
        pygame.init()

        self.screen = pygame.display.set_mode((WIDTH*M2P, HEIGHT*M2P))
        pygame.display.set_caption("Control System Simulation")
        self.background_sprite = scale(load(BACKGROUND_SPRITE), (WIDTH*M2P, HEIGHT*M2P))
        self.rocket_sprite = scale(load(ROCKET_SPRITE), (ROCKET_WIDTH*M2P, ROCKET_HEIGHT*M2P))
        self.fire_sprite = flip(scale(load(FIRE_SPRITE), (FIRE_WIDTH*M2P, FIRE_HEIGHT*M2P)), False, True)
        
        self.rocket = Rocket(locX=WIDTH/2)

        self.reference_points = [] # reference points (xr) to show on screen 
        self.max_ref = 100 # maximum number of reference points stored
        self.rocket_points = [] # actual rocket position (x) to show on screen
        self.max_roc = 100 # maximum number of points stored

        self.draw_ref = draw_reference_line
        self.draw_roc = draw_rocket_line

        self.windX = 0
        self.windZ = 0

        self.update_check = 0

    def reset(self):
        self.rocket = Rocket(locX=WIDTH/2, locZ=ROCKET_HEIGHT/2)

    def setWind(self, x, z):
        self.windX = x
        self.windZ = z

    def get_reference_parameters(self) -> tuple[float, float]:
        """Function that returns the values of vr and yr (reference values for
        the control system to follow)"""
        vr: float = 0.0
        zr: float = 0.0
        
        if self.draw_ref:
            self.reference_points.append((zr, self.rocket.locZ))
            if len(self.reference_points) > self.max_ref:
                del self.reference_points[0]
        return vr, zr

    def draw_object(self, sprite, pos: tuple):
        anchored_pos = (pos[0], HEIGHT*M2P/2 + pos[1] - self.rocket.locZ*M2P)
        self.screen.blit(sprite, anchored_pos)

    def draw_rocket(self):
        fire_rot = self.rocket.theta + 2*self.rocket.nozzleAngle
        fire_height = FIRE_HEIGHT * self.rocket.thrust / MAX_THRUST
        fire = rotate(scale(self.fire_sprite, (FIRE_WIDTH*M2P, fire_height * M2P)), -fire_rot * 180 / pi)
        
        theta = self.rocket.theta
        anchor_pos = (self.rocket.locX*M2P - 0.95 * ROCKET_HEIGHT/2 * M2P * sin(theta) - FIRE_WIDTH/2,
                      self.rocket.locZ*M2P + 0.95 * ROCKET_HEIGHT/2 * M2P * cos(theta))
        final_pos = (anchor_pos[0] - fire_height * M2P / 2 * sin(fire_rot) - fire.get_width()/2,
                     anchor_pos[1] + fire_height * M2P / 2 * cos(fire_rot) - fire.get_height()/2)
        self.draw_object(fire, final_pos)
        rocket = rotate(self.rocket_sprite, -self.rocket.theta * 180 / pi)
        pos = (self.rocket.locX*M2P - rocket.get_width() / 2,
               self.rocket.locZ*M2P - rocket.get_height() / 2)
        self.draw_object(rocket, pos)

    def draw_scenario(self):
        self.screen.blit(self.background_sprite, (0, clip(self.rocket.locZ*M2P, HEIGHT*M2P)))
        self.screen.blit(self.background_sprite, (0, clip(self.rocket.locZ*M2P - HEIGHT*M2P, -HEIGHT*M2P)))
        pygame.draw.rect(self.screen, (0, 255, 0), (0, (HEIGHT/2 + self.rocket.locZ)*M2P, WIDTH*M2P, HEIGHT*M2P))

        
        if self.draw_ref:
            for i in range(1, len(self.reference_points)):
                P1 = self.reference_points[i-1] * M2P
                P1 = P1[0], HEIGHT*M2P/2 - P1[1] + self.rocket.locZ * M2P
                P2 = self.reference_points[i] * M2P
                P2 = P2[0], HEIGHT*M2P/2 - P2[1] + self.rocket.locZ * M2P
                pygame.draw.line(self.screen, RED, P1, P2)

        if self.draw_roc:
            for i in range(1, len(self.rocket_points)):
                P1 = self.rocket_points[i-1] * M2P
                P1 = P1[0], HEIGHT*M2P/2 - P1[1] + self.rocket.locZ * M2P
                P2 = self.rocket_points[i] * M2P
                P2 = P2[0], HEIGHT*M2P/2 - P2[1] + self.rocket.locZ * M2P
                pygame.draw.line(self.screen, WHITE, P1, P2)

        self.draw_rocket()

    def update(self, verbose = False):
        self.screen.fill(BLACK)
        if self.draw_roc:
            self.rocket_points.append((self.rocket.locX, self.rocket.locZ))
            if len(self.rocket_points) > self.max_roc:
                del self.rocket_points[0]

        self.draw_scenario()
        self.rocket.move(self.windX, self.windZ, verbose = verbose)
        if self.rocket.locX < 0 or self.rocket.locX > WIDTH:
            self.reset()
        # if self.rocket.locZ - fabs(cos(self.rocket.theta))*ROCKET_HEIGHT/2 < 0:
        #     self.rocket.locZ = fabs(cos(self.rocket.theta))*ROCKET_HEIGHT/2
        #     self.rocket.speedZ = 0
        pygame.display.flip()
