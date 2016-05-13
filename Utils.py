import random

import gzip
import math
import heapq
import multiprocessing
from itertools import izip
from PIL import Image
import numpy as np
import cPickle as pickle

WORLD_TO_MAP_SCALE = 15.758
RAY_MOD = 20
CELLS_IN_ONE_METRE = 2
LASER_MAX_RANGE = 10
LASER_SCAN_ANGLE_INCREMENT = 10
TOTAL_PARTICLES = 1200
PARTICLES_PER_LANDMARK = TOTAL_PARTICLES / 6

THRESHOLD = 1.0/(TOTAL_PARTICLES * 10.0)

CENTROID_THRESHOLD = TOTAL_PARTICLES * 0.75  # %
RESAMPLE_THRESHOLD = TOTAL_PARTICLES * 0.80  # %
BOUNDING_BOX_AREA_CONVERGENCE = 4            # bounding box with area 4m^2 is considered converged
MAPWIDTH = 2000
MAPHEIGHT = 700

global cspace
cspace = None

class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

def distance_to_obs((mx, my), theta):
    global cspace
    if cspace == None:
        initialize_cspace()
    theta += 270
    idx = int((theta / 2.0)) % 180
    if (my, mx) in cspace:
        return cspace[(my, mx)][idx]

def initialize_cspace():
    global cspace
    try:
        f = open('cspace.pklz','rb')
        cspace = pickle.load(f)
        print("cspace loaded")
        f.close()
    except IOError:
        print("No configuration space calculated! Do that.")

def bresenham(start, end):
    x1, y1 = start
    x1, y1 = int(round(x1)), int(round(y1))
    x2, y2 = end
    x2, y2 = int(round(x2)), int(round(y2))
    dx = x2 - x1
    dy = y2 - y1
    is_steep = abs(dy) > abs(dx)
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
    dx = x2 - x1
    dy = y2 - y1
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
    if swapped:
        points.reverse()
    return points

def get_free_cell(map):
    height = len(map)
    width = len(map[0])

    while True:
        x = int(random.uniform(0, height))
        y = int(random.uniform(0, width))
        if map[x][y] == 255:
            return x, y

def is_free_gcs(x, y, map):
    """
    Checks if (x, y) is free in the global coordinate system.
    """
    mx, my = gcs2map(x, y)
    return is_free_map(mx, my, map)


def is_free_map(mx, my, map):
    """
    Checks if map pixel (mx, my) is free.
    """

    # Check in my, mx because traditionally 2D arrays have X going top-down
    # and Y going left-right. But with images/pixels it is vice-versa.
    # Can't be black
    if mx >= 2000:
        return False
    if my >= 700:
        return False
    return(np.any(map[my, mx] != (0, 0, 0)))

def map2gcs(mx, my):
    """
    Convert map pixels co-ords to global coord system
    """
    x = (mx - 1000.0) / WORLD_TO_MAP_SCALE
    y = (350.0 - my) / WORLD_TO_MAP_SCALE
    return int(x), int(y)

def gcs2map(x, y):
    """
    Convert global coords to map pixels.
    """
    mx = 1000 + x * WORLD_TO_MAP_SCALE
    my = 350 - y * WORLD_TO_MAP_SCALE
    return int(mx), int(my)

def get_endpoint_at_angle(x, y, theta):
    dx = x + LASER_MAX_RANGE * math.cos(math.radians(theta))
    dy = y + LASER_MAX_RANGE * math.sin(math.radians(theta))
    return int(round(dx)), int(round(dy))


def prob_diff_readings(robot, particle):
    """
    Return how similar robot and particle readings are.
    Calculated as % similarity. Exact same readings are 100% similar and will return 1.
    """
    diff = 1.0
    a = 1.0
    c = 0.8
    for expected, actual in zip(robot, particle):
        b = expected
        gaus_m = lambda x: a*math.e**(-1.0*(((x-b)**2.0)/(2.0*c**2)))
        diff *= gaus_m(actual)
    return diff

