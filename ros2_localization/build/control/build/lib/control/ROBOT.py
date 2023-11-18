import numpy as np

# shaft encoder and motor specific
ENCODER = 12*3952/33

# robot dimensions specific
WHEEL_RADIUS = 0.035
BASE_LENGTH = 0.14


# Tentacle planner
V = 1/6.6
W_AC = 2*np.pi/5.2
W_C = 2*np.pi/5.2

DUTY_CYCLES = [(0,0), (0.37, 0.35), (-0.37, -0.35), (-0.18, 0.18), (0.18, -0.18), (0.185, 0.163), (-0.13, 0.13), (0.13, -0.13)]


DUTY_CYCLE_LEFT = 98
DUTY_CYCLE_RIGHT = 100

