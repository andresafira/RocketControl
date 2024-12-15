from math import pi

# World dimensions
M2P = 10 # Meter to Pixel
WIDTH = 150
HEIGHT = 100

#  Sample time parameters
FREQUENCY = 60.0
D_TIME = 1.0 / FREQUENCY

# Sprite locations
BACKGROUND_SPRITE = "sprites/background.jpeg"
ROCKET_SPRITE = "sprites/rocket.png"
FIRE_SPRITE = "sprites/fire.png" 

# Physics paremeters
AIR_RES_X = 100
AIR_RES_Z = 10
INERTIA = 100000
GRAVITY = 9.81
INERTIA_NOZZLE = pi / 180 / 5

# Fire parameters
FIRE_WIDTH = 3
FIRE_HEIGHT = 10

# Rocket parameters
ROCKET_WIDTH = 5
ROCKET_HEIGHT = 20
ROCKET_MASS = 1000
D_NOZZLE_ANGLE = pi / 180 / 2
D_THRUST = 100
MAX_THRUST = 5*ROCKET_MASS*GRAVITY
MAX_NOZZLE_ANGLE = 10*pi / 180
POS_CM = 2*ROCKET_HEIGHT/3
POS_CG = ROCKET_HEIGHT/2

# RGB colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
