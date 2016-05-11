from PIL import Image
import math
import numpy as np
import pickle

MAPWIDTH = 2000
MAPHEIGHT = 700
WORLD_TO_MAP_SCALE = 15.758

LASER_MAX_RANGE = 10.0 # TODO CORRECT NUMBER

print("No configuration space yet! Calculating...")
mapfile = '/home/stu9/s4/bwb5381/project.png'
themap = Image.open(mapfile, mode='r')
themap = themap.convert("RGB")
mappix = themap.load()


def is_free(mx, my):
    """
    Checks if (x, y) is free in the global coordinate system.
    """
    return(np.any(mappix[my, mx] != (0, 0, 0)))

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

cspace = dict()
for x in range(0, MAPHEIGHT):
    print("Calculating for row %d"%(x))
    if x not in cspace:
        cspace[x] = dict()
    for y in range(0, MAPWIDTH):
        if y not in cspace[x]:
            cspace[x][y] = dict()
        wc_x, wc_y = map2gcs(x, y)
        for theta in range(0, 360, 2):
            if is_free(x, y):
                dist = LASER_MAX_RANGE
                wc_dx = wc_x + LASER_MAX_RANGE * math.cos(math.radians(theta))
                wc_dy = wc_y + LASER_MAX_RANGE * math.sin(math.radians(theta))
                dx, dy = gcs2map(wc_dx, wc_dy)
                bp = bresenham((x, y), (dx, dy))
                for bpx, bpy in bp:
                    if not is_free(bpx, bpy):
                        wc_bpx, wc_bpy = map2gcs(bpx, bpy)
                        dist = math.sqrt((wc_x - wc_bpx) ** 2 + (wc_y - wc_bpy) ** 2)
                        break
                cspace[x][y][theta] = dist
pickle.dump(cspace, open("cspace.p", "wb"))

