import math
from Utils import *
import numpy as np

SCATTER_SPREAD = 0.5
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
        sample_angles = [(dtheta * 0.09083)+ self.theta for dtheta in xrange(-320, 320, RAY_MOD)]
        ray_endpts = [get_endpoint_at_angle(self.x, self.y, a) for a in sample_angles]
        readings = []

        for i, ep in enumerate(ray_endpts):
            #scaled_ep = ep[0] * CELLS_IN_ONE_METRE, ep[1] * CELLS_IN_ONE_METRE
            #bp = bresenham((self.x, self.y), scaled_ep)
            scaled_ep = gcs2map(ep[0], ep[1])
            bp = bresenham((self.mx, self.my), scaled_ep)
            obstacle_found = False
            for bpx, bpy in bp:
                if not is_free_map(bpx, bpy, map):
                    obstacle_found = True
                    readings.append(self.map_get_distance_to(bpx, bpy))
                    break
            if not obstacle_found:
                readings.append(LASER_MAX_RANGE)
        return readings

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

