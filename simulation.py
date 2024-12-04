from constants import CAR_START_LEFT, CAR_START_RIGHT, CAR_MAX_SPEED, SAFETY_DISTANCE_BACK, SAFETY_DISTANCE_FRONT
from constants import WIDTH, HEIGHT, CAR_HEIGHT, CAR_WIDTH, BACKGROUND_SPRITE, CAR_SPRITE
from constants import SAMPLE_TIME, eps, SIDEWALK_WIDTH, MIDDLE_RIGHT, MIDDLE_LEFT
from constants import BLACK, WHITE, RED

from Utils.Geometry.Position import Position
from Utils.General import clip
from Utils.Geometry.Box import *

import pygame
from pygame.transform import scale, rotate
from pygame.image import load
from pygame.locals import *

from math import pi, sin, cos, fabs
from Car import Car


def make_default_position(location):
    return Position(Vector(location[0], location[1]), 0)


class Simulation:
    """Main simulation engine that manages the interaction between objects"""
    def __init__(self, side: str, draw_Bounding_Box: bool, draw_reference_line: bool, draw_car_line: bool):
        pygame.init()

        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("Control System Simulation")
        self.background_sprite = load(BACKGROUND_SPRITE)
        self.car_sprite = scale(load(CAR_SPRITE), (CAR_WIDTH*1.1, CAR_HEIGHT))

        if side == 'left':
            self.car = Car(make_default_position(CAR_START_LEFT))
        elif side == 'right':
            self.car = Car(make_default_position(CAR_START_RIGHT))
        else:
            raise Exception('Invalid side choice! Choose left or right side')

        self.dummies = [] # stores the dummy cars
        self.objects = [] # stores the objects in the simulation

        self.reference_points = [] # reference points (xr) to show on screen 
        self.max_rp = 100 # maximum number of reference points stored
        self.car_points = [] # actual car position (x) to show on screen
        self.max_cp = 100 # maximum number of points stored

        self.draw_BB = draw_Bounding_Box
        self.draw_RL = draw_reference_line
        self.draw_CL = draw_car_line

        self.update_check = 0

    def dummy_simple_generator(self, num_dummy, step, side, speed):
        """Method that generates an alternating lanes set of dummy cars with
        specified number, speed, initial side and distance between."""

        if side == 'left':
            first = MIDDLE_RIGHT
            second = MIDDLE_LEFT
        elif side == 'right':
            first = MIDDLE_LEFT
            second = MIDDLE_RIGHT
        else:
            raise Exception('Invalid side choice! Choose left or right side')

        self.dummies.clear()
        for i in range(num_dummy):
            if i % 2 == 0:
                x = first
            else:
                x = second
            pose = Position(Vector(x, HEIGHT/2 - CAR_HEIGHT + i * step), 0)
            self.dummies.append(Car(initial_position=pose, DUMMY=True, initial_speed=speed))
    
    def reset(self, side: str):
        self.car.speed = 0
        self.car.alive = True
        self.dummies.clear()
        self.update_check = 0
        if side == 'left':
            self.car.position = make_default_position(CAR_START_LEFT)
        elif side == 'right':
            self.car.position = make_default_position(CAR_START_RIGHT)
        else:
            raise Exception('Invalid side choice! Choose between left or right side')
        self.update_objects()

    def get_reference_parameters(self) -> tuple[float, float]:
        """Function that returns the values of vr and yr (reference values for
        the control system to follow)"""
        vr = CAR_MAX_SPEED
        xr = MIDDLE_RIGHT

        # If there is a dummy car ahead (given a safety threshold) the reference point goes
        # to the left lane, and, after it passes over (given another safety threshold) the 
        # reference point goes back to the right lane
        for dummy in self.dummies:
            delta_y = dummy.position.location.y - self.car.position.location.y
            if delta_y > 0 and delta_y < SAFETY_DISTANCE_BACK or delta_y < 0 and -delta_y < SAFETY_DISTANCE_FRONT:
                xr = MIDDLE_LEFT
                break
        
        if (self.draw_RL):
            self.reference_points.append((xr, self.car.position.location.y))
            if len(self.reference_points) > self.max_rp:
                del self.reference_points[0]
        return vr, xr

    def generate_dummy(self, x_position, speed):
        pose = Position(Vector(x_position, self.car.position.location.y + CAR_HEIGHT/2 + HEIGHT/2), 0)
        car = Car(initial_position=pose, DUMMY=True, initial_speed=speed)
        self.dummies.append(car)

    def update_objects(self):
        self.objects.clear()
        for dummy in self.dummies:
            dummy.update()

        # ROAD Segment:
        left_side = Segment(Vector(SIDEWALK_WIDTH, self.car.position.location.y - CAR_HEIGHT),
                            Vector(SIDEWALK_WIDTH, self.car.position.location.y + CAR_HEIGHT))
        right_side = Segment(Vector(WIDTH - SIDEWALK_WIDTH, self.car.position.location.y - CAR_HEIGHT),
                             Vector(WIDTH - SIDEWALK_WIDTH, self.car.position.location.y + CAR_HEIGHT))

        self.objects.append(left_side)
        self.objects.append(right_side)
        for dummy in self.dummies:
            self.objects.append(dummy.bounding_box)

    def draw_bounding_box(self, car: Car):
        box = car.unravel_box()
        P1 = box.vertices[0].x, HEIGHT/2 - box.vertices[0].y + self.car.position.location.y + CAR_HEIGHT / 2
        P2 = box.vertices[1].x, HEIGHT/2 - box.vertices[1].y + self.car.position.location.y + CAR_HEIGHT / 2
        P3 = box.vertices[2].x, HEIGHT/2 - box.vertices[2].y + self.car.position.location.y + CAR_HEIGHT / 2
        P4 = box.vertices[3].x, HEIGHT/2 - box.vertices[3].y + self.car.position.location.y + CAR_HEIGHT / 2

        pygame.draw.line(self.screen, WHITE, P1, P2)
        pygame.draw.line(self.screen, WHITE, P2, P3)
        pygame.draw.line(self.screen, WHITE, P3, P4)
        pygame.draw.line(self.screen, WHITE, P4, P1)

    def draw_car(self, car: Car):
        sprite_copy = rotate(self.car_sprite, -car.position.rotation * 180 / pi)
        if not self.car.alive:
            sprite_copy.set_alpha(50)
        self.screen.blit(sprite_copy, (car.position.location.x - sprite_copy.get_width() / 2,
                                       HEIGHT / 2 + CAR_HEIGHT/2 - sprite_copy.get_height() / 2 - car.position.location.y + self.car.position.location.y))
        if self.draw_BB:
            self.draw_bounding_box(car)

    def draw_scenario(self):
        self.screen.blit(self.background_sprite, (0, clip(self.car.position.location.y, HEIGHT)))
        self.screen.blit(self.background_sprite, (0, clip(self.car.position.location.y - HEIGHT, -HEIGHT)))
        for dummy in self.dummies:
            self.draw_car(dummy)
        
        if self.draw_RL:
            for i in range(1, len(self.reference_points)):
                P1 = self.reference_points[i-1]
                P1 = P1[0], HEIGHT/2 - P1[1] + self.car.position.location.y
                P2 = self.reference_points[i]
                P2 = P2[0], HEIGHT/2 - P2[1] + self.car.position.location.y
                pygame.draw.line(self.screen, RED, P1, P2)

        if self.draw_CL:
            for i in range(1, len(self.car_points)):
                P1 = self.car_points[i-1]
                P1 = P1[0], HEIGHT/2 - P1[1] + self.car.position.location.y
                P2 = self.car_points[i]
                P2 = P2[0], HEIGHT/2 - P2[1] + self.car.position.location.y
                pygame.draw.line(self.screen, WHITE, P1, P2)

        self.draw_car(self.car)

    def update(self):
        self.screen.fill(BLACK)
        if self.update_check != 0:
            self.objects.clear()
            self.update_check = (self.update_check + 1) % 3

        # Removing cars that are behind
        i = 0
        while i < len(self.dummies):
            if self.car.position.location.y - self.dummies[i].position.location.y > HEIGHT:
                del self.dummies[i]
            else:
                i += 1

        self.car.update(self.objects)
        if (self.draw_CL):
            self.car_points.append((self.car.position.location.x, self.car.position.location.y))
            if len(self.car_points) > self.max_cp:
                del self.car_points[0]

        self.draw_scenario()
        self.update_objects()
        pygame.display.flip()
