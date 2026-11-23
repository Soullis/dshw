import numpy as np

TAKEOFF_ALT = 1.5
TAKEOFF_MAX_TRIES = 5

GOAL_M = np.array([5.0, 0.0, 0.0])
K_ATT = 0.6
K_REP = 1.2
D0 = 1.5
V_MAX = 0.15
GOAL_TOL = 0.25
TIMEOUT = 360.0
LOOP_HZ = 20.0
OBSTACLE_NS = "obstacle_detection"

WIDTH = 424
HEIGHT = 240

DEVICE = 0
