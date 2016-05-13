import math
from Utils import *
import numpy as np

SCATTER_SPREAD = 0.8
ANGLE_NOISE = 10.0

class Particle:
    def __init__(self, x, y, theta=None, p=0.0005):
        if theta is None:
            theta = random.uniform(0, 360)
        # Global coordinate system
        self.x, self.y, self.theta = x, y, theta
        # Map pixels
        self.mx, self.my = gcs2map(self.x, self.y)
        # probability
        self.p = p

    def to_string(self):
        return '({0}, {1}), theta: {2}, p:{3}'.format(self.x, self.y, self.theta, self.p)

    def __str__(self):
        return self.to_string()

    def __repr__(self):
        return self.to_string()

    @staticmethod
    def particles_around_point(particles_per_ladmark, landmarks, map, maintain_start_angle=True):
        p = []
        while len(p) != particles_per_ladmark:
            x = landmarks[0] + np.random.normal(0.0, SCATTER_SPREAD)
            y = landmarks[1] + np.random.normal(0.0, SCATTER_SPREAD)
            theta = landmarks[2] + np.random.normal(0.0, ANGLE_NOISE) if maintain_start_angle else None

            if is_free_gcs(x, y, map):
                p.append(Particle(x, y, theta, p=1.0/(particles_per_ladmark * len(landmarks))))
        return p

    @staticmethod
    def scatter_near_test(count, landmarks, map, maintain_start_angle=False):
        p = []
        for l in landmarks:
            x = l[0]
            y = l[1]
            theta = l[2]
            p.append(Particle(x, y, theta, p=1.0/(100 * len(landmarks))))
            #x = l[0] + 1.0
            #y = l[1] + 1.0
            #theta = l[2] + 1.0
            #p.append(Particle(x, y, theta, p=1.0/(100 * len(landmarks))))
        return p

    @staticmethod
    def scatter_around_map(count, map):
        p = []
        while len(p) != count:
            y = np.random.randint(0, 700)
            x = np.random.randint(0, 2000)
            (x, y) = map2gcs(x, y)
            theta = np.random.randint(0, 360)
            if is_free_gcs(x, y, map):
                p.append(Particle(x, y, theta, p=1.0/count))
        return p

    @staticmethod
    def scatter_near_landmarks(count, landmarks, map, maintain_start_angle=False):
        p = []
        for l in landmarks:
            p += Particle.particles_around_point(count, l, map, maintain_start_angle)
        return p

    def move(self, dist, dtheta, map):
        """
        Move particle in direction it is facing. Some noise is added to the particle's
        motion as well. Particle self-checks to see if it is in walls or in an "impossible"
        location and updates own probability to 0.
        """
        # todo add some noise here
        self.theta += dtheta
        r = math.radians(self.theta)
        dx, dy = math.cos(r) * dist, math.sin(r) * dist
        self.x += dx
        self.y += dy
        self.mx, self.my = gcs2map(self.x, self.y)

        # Let particle update it's own probability if it's not in free space.
        if not is_free_map(self.mx, self.my, map):
            self.p = 0

    def sense(self, map):
        # TODO MAKE MORE ACCURATE
        sample_angles = [360 - (dtheta + self.theta) for dtheta in range(-29, 29, 2)]
        readings = []
        locs = []
        for i, angle in enumerate(sample_angles):
            dist = distance_to_obs((self.mx, self.my), angle)
            if dist != None:
                readings.append(dist)
                loc = (self.mx + (dist*WORLD_TO_MAP_SCALE)*math.cos(np.radians(angle)), self.my + (dist*WORLD_TO_MAP_SCALE)*math.sin(np.radians(angle)))
                locs.append(loc)
        return readings, locs

    def map_get_distance_to(self, j, k):
        return math.sqrt((self.mx - j) ** 2 + (self.my - k) ** 2) / WORLD_TO_MAP_SCALE

    def gcs_get_distance_to(self, j, k):
        return math.sqrt((self.x - j) ** 2 + (self.y - k) ** 2)

    def d3_get_distance_to(self, j, k, l):
        theta_diff = ((l - self.theta) + 180.0 % 360) - 180.0
        return math.sqrt((self.x - j) ** 2 + (self.y - k) ** 2 + (theta_diff ** 2))

    def clone(self, map, with_noise=True):
        for _ in range(100):
            dtheta = self.theta + np.random.normal(0.0, 0.5) if with_noise else 0
            dtheta = ((dtheta + 180) % 360.0) - 180.0
            dx = self.x + np.random.normal(0.0, 0.3) if with_noise else 0
            dy = self.y + np.random.normal(0.0, 0.3) if with_noise else 0
            if is_free_gcs(dx, dy, map):
                return Particle(dx, dy, dtheta, self.p)

        # Should pretty much never happen.
        print 'Tried 100 times. All particles in wall. Returning same particle with angle noise'
        return Particle(self.x, self.y, theta=(self.theta + random.uniform(-30, 30)))

