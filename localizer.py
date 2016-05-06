#!/usr/bin/python

import Tkinter as tk
import ImageTk
import bisect
import numpy as np
from PIL import Image
import random
import rospy
import tf
import math
import Queue
import itertools
import copy

from threading import Timer
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from p2os_msgs.msg import SonarArray
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String
from functools import partial
from multiprocessing import Pool

from Particle import *

def oval(x, y):
    return x - 1, y - 1, x + 1, y + 1

class Localizer(tk.Frame):
    def update_image(self):
        self.mapimage = ImageTk.PhotoImage(self.themap)
        self.canvas.create_image(2000 / 2, 700 / 2, image=self.mapimage)

    def render_particles(self, root=None):
        if self.no_flash_gui:
            self.canvas.delete("particles")
            for p in self.particles:
                self.canvas.create_oval(oval(p.mx, p.my), tags="particles")
            root.after(500, self.render_particles, root)
        else:
            self.reset_map()
            for p in self.particles:
                self.mappix[p.mx, p.my] = 128
            self.update_image()

    def reset_map(self):
        self.themap = Image.open(self.mapfile, mode='r')
        self.mapimage = ImageTk.PhotoImage(self.themap)
        self.mappix = self.themap.load()
        self.update_image()

    def __init__(self, supplied_map, *args, **kwargs):
        tk.Frame.__init__(self, *args, **kwargs)
        self.master.title("Localizer")
        self.master.minsize(width=2000, height=700)
        self.mapfile = supplied_map
        self.themap = Image.open(self.mapfile, mode='r')
        self.mapimage = ImageTk.PhotoImage(self.themap)
        self.mappix = self.themap.load()
        self.canvas = tk.Canvas(self, width=2000, height=700)
        self.map_on_canvas = self.canvas.create_image(2000 / 2, 700 / 2, image=self.mapimage)
        self.canvas.pack()
        self.pack()
        self.render_flag = True
        self.no_flash_gui = False

        self.maparr = np.asarray(self.themap)
        self.landmarks = [[-12.0, 12.0, 180], [10.8, 12.7, 180], [8, -0.5, 90], [-18.4, -8.9, 0], [-54.5, 7.6, 90], [8, -1.5, 270]]
        #self.landmarks = [[10.8, 12.7, 180], [8, -0.5, 90]]
        self.particles = Particle.scatter_near_landmarks(PARTICLES_PER_LANDMARK, self.landmarks, self.maparr, maintain_start_angle=False)
        self.render_particles()

        # odom
        self.ang_delta = 0
        self.fwd_delta = 0
        self.last_theta = 0.0
        self.last_x = 0.0
        self.last_y = 0.0

        # sensors
        self.laser_queue = Queue.LifoQueue()
        self.odom_queue = Queue.LifoQueue()

        self.count = 0
        self.goal_pub = rospy.Publisher('/endgoal', String, queue_size=1)

        self.task_pool = Pool(processes=5)



    def resample(self, count):
        """
        Re-samples particles. Function gets rid of dead particles, normalizes probabilities
        and then spreads out the probs in a [0,1] number line as sorted cumulative list.
        Pick a random float in [0, 1] and use the corresponding particle at index to get a new particle.
        Particles with higher probs have a larger "share" in the line and have a larger chance of being cloned.

        :param count: the number of particles to resample.
        :return:
        """

        print 'Resampling ', count, 'new particles'
        self.normalize_particles()

        # Cumulative probabilities for weighted distribution
        cumul_probs = [_.p for _ in self.particles]
        cumul_probs = np.cumsum(cumul_probs)

        new_particles = []

        # Re-sample particles with probability equivalent to weights
        for _ in xrange(count):
            rand_p = random.uniform(0, 1)
            i = bisect.bisect_left(cumul_probs, rand_p)
            try:
                new_particles.append(self.particles[i].clone(self.maparr, with_noise=True))
            except IndexError:
                pass
                #print 'Index Error WTF?!?!', i

        self.particles += new_particles

    def normalize_particles(self):
        sump = sum(_.p for _ in self.particles)
        for i in xrange(len(self.particles)):
            self.particles[i].p /= sump

    def remove_dead_particles(self, strategy=None):
        self.particles = [p for p in self.particles if p.p > THRESHOLD]

    def odom_update(self, omsg):
        self.odom_queue.put(omsg)

    def get_movement(self):
        omsg = self.odom_queue.get()
        if omsg is None:
            return None
        self.odom_queue.queue[:] = []

        x = omsg.pose.pose.position.x
        y = omsg.pose.pose.position.y
        q = (omsg.pose.pose.orientation.x,
             omsg.pose.pose.orientation.y,
             omsg.pose.pose.orientation.z,
             omsg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(q)
        d_theta = (self.last_theta - euler[2])
        d_theta = (d_theta + math.pi % (2.0 * math.pi)) - math.pi
        # print 'last', self.last_theta, 'new', euler[2], 'dtheta', d_theta
        dx = self.last_x - x
        dy = self.last_y - y
        dist = math.sqrt(dx**2 + dy**2)

        dist = dist if dist > 0.15 else 0
        if dist > 0:
            self.last_x = x
            self.last_y = y

        if abs(d_theta) < 0.08:
            d_theta = 0
        else:
            self.last_theta = euler[2]

        # if abs(d_theta) < 0.001:
        #     d_theta = 0
        # if dx < 0.01:
        #     dx = 0
        # if dy < 0.01:
        #     dy = 0

        # todo -dtheta is hack. robot was turning in the opposite direction. Fix needed
        return dist, -d_theta

    def update(self):
        count = 0
        while True:
            start_time = rospy.get_time()
            count += 1
            mov = self.get_movement()
            if mov is None:
                continue

            dist, dtheta = mov[0], mov[1]

            if dist == 0 and dtheta == 0:
                continue

            for p in self.particles:
                p.move(dist, math.degrees(dtheta), self.maparr)

            lmsg = self.get_laser_msg()
            if lmsg is None:
                continue

            rread = self.get_robot_readings(lmsg)
            rread = [r for idx, r in enumerate(rread) if idx % RAY_MOD == 0]

            self.particles = self.task_pool.map(update_particle,
                                zip(self.particles, itertools.cycle([rread]),
                                    itertools.cycle([self.maparr])))

            end_time = rospy.get_time()
            print("Time taken %d seconds"%(end_time -start_time))

            centroid = self.converged_loc(strategy="centroid")
            if centroid != None:
                print 'Converged !'
                goal_str = "%f %f" %centroid
                self.goal_pub.publish(goal_str)

            self.remove_dead_particles()
            self.normalize_particles()
            self.resample_if_required()
            self.render_particles()
    
    def get_laser_msg(self):
        msg = self.laser_queue.get()
        self.laser_queue.queue[:] = []
        return msg

    def get_robot_readings(self, lmsg):
        indices = [0, 106, 215, 320, 425, 635]
        msg = self.laser_queue.get()
        r = [msg.ranges[i] for i in indices]
        return r

    def resample_if_required(self):
        if len(self.particles) < RESAMPLE_THRESHOLD:
            self.resample(TOTAL_PARTICLES - len(self.particles))

    def converged_loc(self, strategy="centroid"):

        if strategy == "centroid":
            coords = np.asarray([(p.x, p.y) for p in self.particles])
            sum_x = np.sum(coords[:, 0])
            sum_y = np.sum(coords[:, 1])
            centroid = 1.0 * sum_x / len(coords), 1.0 * sum_y / len(coords)
            particles_near_centroid = len([1 for p in self.particles if p.gcs_get_distance_to(*centroid) < 1])

            if particles_near_centroid > CENTROID_THRESHOLD:
                print 'converged about ', centroid
                return centroid
            else:
                return None

        else:
            # Bounding box. Is not complete. How to eliminate outliers?
            min_x, max_x, min_y, max_y = float('inf'), float('-inf'), float('inf'), float('-inf')
            for p in self.particles:
                if p.p == 0:
                    continue
                min_x = min(min_x, p.x)
                max_x = max(max_x, p.x)
                min_y = min(min_y, p.y)
                max_y = max(max_y, p.y)
            width = max_x - min_x
            height = max_y - min_y
            area = width * height

            return area < BOUNDING_BOX_AREA_CONVERGENCE

    def laser_update(self, lmsg):
        self.laser_queue.put(lmsg)

def update_particle((p, rread, maparr)):
    #np = Particle(p.x, p.y, p.theta, p.p)
    if p.p != 0:
        pread = p.sense(maparr)
        p.p *= prob_diff_readings(rread, pread)
    return p

def main():
    rospy.init_node("localize", anonymous=True)
    root = tk.Tk()
    l = Localizer('/home/stu9/s4/bwb5381/project.png', master=root, height=700, width=2000)
    rospy.Subscriber("/r1/kinect_laser/scan", LaserScan, l.laser_update)
    rospy.Subscriber("/r1/odom", Odometry, l.odom_update)
    t = Timer(0.1, l.update)
    t.start()
    root.mainloop()


if __name__ == "__main__":
    main()
