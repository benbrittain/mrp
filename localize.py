#!/usr/bin/python

import Tkinter as tk
from PIL import Image
import ImageTk

import colorsys

import random
import rospy
import tf
import math
import Queue
import sys
from threading import Timer
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from p2os_msgs.msg import SonarArray
from geometry_msgs.msg import Quaternion

MAPWIDTH = 2000
MAPHEIGHT = 700
PARTICLE_COUNT = 200

PIXELSIZE = 0.06346

def w2m((x, y)):
    map_x = x / PIXELSIZE
    map_y = y / PIXELSIZE
    map_x = map_x + math.floor(MAPWIDTH/2)
    map_y = map_y + math.floor(MAPHEIGHT/2)
    return (map_x, map_y)

def m2w((map_x, map_y)):
    world_y = map_y - MAPHEIGHT/2.0
    world_y = world_y * PIXELSIZE
    world_x = map_x - MAPWIDTH/2.0
    world_x = world_x * PIXELSIZE
    return (world_x, world_y)

def distance((x1, y1)):
    return math.sqrt((x1)**2 + (y1)**2)

#TODO merge
def dist((x0, y0), (x1, y1)):
    return math.sqrt((x1-x0)**2 + (y1-y0)**2)

def normalize(particles):
    prob_factor = 1.0 / sum(a.prob for a in particles)
    for x in particles:
        x.prob = prob_factor * x.prob

class Particle(object):
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.prob = 1.0 / PARTICLE_COUNT

    def propogate(self, fwd_delta, ang_delta):
        self.theta += ang_delta
        self.theta = ((self.theta + math.pi) % (2*math.pi)) - math.pi
        self.x += fwd_delta * math.cos(self.theta)
        self.y += fwd_delta * math.sin(self.theta)

    # measuremnt = [(actual_distance, expected_distance)]
    def update_prob(self, measurements):
        new_prob = 1.0
        for m in measurements:
            a = 1.0
            b = m[1]
            c = 1.7
            gaus_m = lambda x: a*math.e**(-1.0*(((x-b)**2.0)/(2.0*c**2)))
            new_prob *= gaus_m(m[0])
        self.prob = self.prob * new_prob

class Localizer(tk.Frame):    
    def __init__(self, *args, **kwargs):
        tk.Frame.__init__(self, *args, **kwargs)
        self.master.title("localizer")
        self.master.minsize(width=MAPWIDTH,height=MAPHEIGHT)
        self.canvas = tk.Canvas(self,width=MAPWIDTH, height=MAPHEIGHT)
        self.canvas.pack()

        self.themap = Image.open("/home/stu9/s4/bwb5381/project.png")
        self.themap = self.themap.convert("RGB")
        self.mappix = self.themap.load()

        self.wallmap = Image.open("/home/stu9/s4/bwb5381/project.png")
        self.wallmap = self.themap.convert("RGB")
        self.wallmappix = self.themap.load()


        self.mapimage = ImageTk.PhotoImage(self.themap)
        self.canvas.create_image(MAPWIDTH/2, MAPHEIGHT/2, image = self.mapimage)
        self.pack()

        # odom
        self.ang_delta = 0
        self.fwd_delta = 0
        self.particles = []
        self.last_theta = 0.0
        self.last_x = 0.0
        self.last_y = 0.0

        # sensors
        self.laser_queue = Queue.LifoQueue()

        # generate particles
        for x in range(PARTICLE_COUNT):

            rh = random.randint(0, MAPHEIGHT-1)
            rw = random.randint(0, MAPWIDTH-1)
            while (self.mappix[rw, rh] == (0, 0, 0)):
                rh = random.randint(0, MAPHEIGHT-1)
                rw = random.randint(0, MAPWIDTH-1)

            costheta = random.uniform(-1,1)
            rt = math.acos(costheta)
            rt = math.pi
            (rw, rh) = m2w((rw, rh))
            print("adding particle at %f,%f@%f"%(rw,rh,rt))
            self.particles.append(Particle(rw,rh,rt))

    def update_image(self):
        self.mapimage = ImageTk.PhotoImage(self.themap)
        self.canvas.create_image(MAPWIDTH/2, MAPHEIGHT/2, image = self.mapimage)

        self.themap = Image.open("/home/stu9/s4/bwb5381/project.png")
        self.themap = self.themap.convert("RGB")
        self.mappix = self.themap.load()

    def draw_particles(self):
        for p in self.particles:
            h = 0.33 * p.prob
            r, g, b = colorsys.hls_to_rgb(h, 127, -1)
            color = (int(r), int(g), int(b))
            x, y = w2m((p.x, p.y))
            self.mappix[x+1, y] = color
            self.mappix[x, y+1] = color
            self.mappix[x+1, y+1] = color
            self.mappix[x, y] = color

        self.after(0, self.update_image)

    def odom_update(self, omsg):
        #print("updating change")
        x = omsg.pose.pose.position.x
        y = omsg.pose.pose.position.y

        q = (omsg.pose.pose.orientation.x,
             omsg.pose.pose.orientation.y,
             omsg.pose.pose.orientation.z,
             omsg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(q)
        d_theta = (self.last_theta - euler[2])
        d_theta = (d_theta + math.pi % (2.0 * math.pi)) - math.pi
        self.last_theta = euler[2]


        dx = self.last_x - x
        dy = self.last_y - y

        self.last_x = x
        self.last_y = y

        if abs(d_theta) < 0.001:
            d_theta = 0
        if dx < 0.01:
            dx = 0
        if dy < 0.01:
            dy = 0
        for p in self.particles:
            p.propogate(distance((dx,dy)), d_theta)
        self.draw_particles()

    def find_nearest_obs(self, start_x, start_y, theta):
        end_x = int(start_x + (10.0) * math.cos(theta))
        end_y = int(start_y + (10.0) * math.sin(theta))

        m_start_x, m_start_y = map(int, w2m((start_x,start_y)))
        m_end_x, m_end_y = map(int, w2m((end_x, end_y)))

        ## the world's laziest raytracing, already had the code
        pixels = bresenham((m_start_x, m_start_y), (m_end_x, m_end_y))
        for (x, y) in pixels[3:]:
            color = (0, 0, 255)
            if (self.wallmappix[x, y] != (0, 0, 0)):
                self.mappix[x, y] = color
            if (self.wallmappix[x, y] == (0, 0, 0)):
                return dist((start_x, start_y), m2w((x, y)))
        return(10.0)

    def resample(self):
        cutoff = (0.5 * (1.0 / PARTICLE_COUNT))
        self.particles = [x for x in self.particles if x.prob > cutoff]
        while len(self.particles) < PARTICLE_COUNT:
            pidx = int(random.uniform(0, len(self.particles)))
            p = self.particles[pidx]
            np = Particle(p.x, p.y, p.theta)
            np.prob = p.prob
            self.particles.append(np)

    def update(self):
        while True:
            lmsg = self.get_queue(self.laser_queue)
            if lmsg != None:
                for p in self.particles:
                    measurements = []
                    for i in range(0, len(lmsg.ranges)):
                        range_theta = lmsg.angle_min + (i * lmsg.angle_increment)
                        range_dist = lmsg.ranges[i]
                        theta = (p.theta + range_theta)
                        expected_dist = self.find_nearest_obs(p.x, p.y, theta)
                        measurements.append((range_dist, expected_dist))
                        #print(range_dist, expected_dist)
                    p.update_prob(measurements)
                    normalize(self.particles)
                    #for x in self.particles:
                    #    print x.prob
                    self.resample()
        #self.after(0, self.update_image)

    def get_queue(self, queue):
        if not queue.empty():
            msg = queue.get()
            while not queue.empty():
                try:
                    queue.get(False)
                except Empty:
                    continue
                queue.task_done()
            return msg

    def laser_update(self,lmsg):
        self.laser_queue.put(lmsg)

def bresenham(start, end):
    x1, y1 = start
    x2, y2 = end
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

def main():
    
    rospy.init_node("localize", anonymous=True)
    root = tk.Tk()
    loc = Localizer(master=root, height=MAPHEIGHT, width=MAPWIDTH)

    rospy.Subscriber("/r1/kinect_laser/scan",LaserScan,loc.laser_update)
    #rospy.Subscriber("/sonar",SonarArray,m.sonar_update)

    #rospy.Subscriber("/pose",Odometry,m.loc_update)
    rospy.Subscriber("/r1/odom",Odometry,loc.odom_update)
    t = Timer(0.1, loc.update)
    t.start()
    root.mainloop()

if __name__ == "__main__":
    main()
