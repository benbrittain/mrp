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
import colorsys

from threading import Timer
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from p2os_msgs.msg import SonarArray
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String
from functools import partial
from multiprocessing import Pool

from Utils import initialize_cspace
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
            for p in self.obs_points:
                r, g, b = (255, 100, 0)
                color = (int(r), int(g), int(b))
                self.mappix[p[0], p[1]] = color
                self.mappix[p[0]+1, p[1]] = color
                self.mappix[p[0]+1, p[1]+1] = color
                self.mappix[p[0], p[1]+1] = color
            self.obs_points = []
            for p in self.particles:
                h = 0.33 * p.p
                r, g, b = colorsys.hls_to_rgb(h, 127, -1)
                color = (int(r), int(g), int(b))
                self.mappix[p.mx, p.my] = color
            self.update_image()

    def reset_map(self):
        self.themap = Image.open(self.mapfile, mode='r')
        self.themap = self.themap.convert("RGB")
        self.mapimage = ImageTk.PhotoImage(self.themap)
        self.mappix = self.themap.load()
        self.update_image()

    def __init__(self, supplied_map, *args, **kwargs):
        tk.Frame.__init__(self, *args, **kwargs)
        self.master.title("Localizer")
        self.master.minsize(width=2000, height=700)
        self.mapfile = supplied_map
        self.themap = Image.open(self.mapfile, mode='r')
        self.themap = self.themap.convert("RGB")

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
        #self.landmarks = [[8, -1.5, 270]]
        #self.landmarks = [[-54.5, 7.6, 90]]
        #self.landmarks = [[10.8, 12.7, 180], [8, -1.5, 270]]
        #self.particles = Particle.scatter_near_test(100, self.landmarks, self.maparr, maintain_start_angle=True)
        self.particles = Particle.scatter_near_landmarks(PARTICLES_PER_LANDMARK, self.landmarks, self.maparr, maintain_start_angle=True)
        self.obs_points = []
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
        self.goal_pub = rospy.Publisher('/localized_pos', String, queue_size=1)

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
        # Re-sample particles with probability equivalent to weights
        for _ in range(count):
            rand_p = random.uniform(0, 1)
            i = bisect.bisect_left(cumul_probs, rand_p)
            try:
                self.particles.append(self.particles[i].clone(self.maparr, with_noise=True))
            except IndexError:
                pass

    def normalize_particles(self):
        min_x = min(_.p for _ in self.particles)
        max_x = max(_.p for _ in self.particles)
        if min_x != max_x:
            for i in range(len(self.particles)):
                self.particles[i].p = (self.particles[i].p - min_x) / (max_x - min_x)

    def remove_dead_particles(self, strategy=None):
        self.particles = filter(lambda p: p.p > THRESHOLD, self.particles)

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

        # todo -dtheta is hack. robot was turning in the opposite direction. Fix needed
        return (x, y), dist, -d_theta, euler[2]

    def update(self):
        count = 0
        while True:
            start_time = rospy.get_time()
            count += 1
            mov = self.get_movement()
            if mov is None:
                continue

	    ox, oy = mov[0][0], mov[0][1]
            dist, dtheta = mov[1], mov[2]
	    otheta = mov[3]
		
            if dist == 0 and dtheta == 0:
                continue

            for p in self.particles:
                p.move(dist, math.degrees(dtheta), self.maparr)

            lmsg = self.get_laser_msg()
            if lmsg is None:
                continue

            readings = self.get_robot_readings(lmsg)
            step = 0.00158544606529
            idx = [int(np.radians(dtheta)/step) for dtheta in range(0, 60, 2)]
            rread = []
            for i in idx:
                rread.append(readings[i])

            for p in self.particles:
                pread, locs = p.sense(self.maparr)
                self.obs_points += locs
                p.p *= prob_diff_readings(rread, pread)

            end_time = rospy.get_time()
            print("Time taken %d seconds"%(end_time -start_time))

            centroid = self.converged_loc(strategy="centroid")
            if centroid is not None:

	        extra_mov = self.get_movement()
		nx, ny = extra_mov[0][0], extra_mov[0][1]
		ntheta = extra_mov[3]		
		
                median_angle = math.radians(np.median([p.theta for p in self.particles])) % (2.0 * math.pi)
		lx, ly = centroid[0], centroid[1]
		xx = median_angle - otheta
		adjust_x = nx * np.cos(xx) - ny * np.sin(xx) + lx
		adjust_y = ny * np.sin(xx) + ny * np.cos(xx) + ly
		adjust_theta = ntheta + xx
		
                print("Converged at <%0.2f,%0.2f>@%0.2f"%(centroid[0], centroid[1], median_angle))
                print("Adjusted convergence at <%0.2f,%0.2f>@%0.2f"%(adjust_x, adjust_y, adjust_theta))
		#goal_str = '{0} {1} {2}'.format(centroid[0], centroid[1], median_angle)
                goal_str = '{0} {1} {2}'.format(adjust_x, adjust_y, xx)
                self.goal_pub.publish(goal_str)

            self.normalize_particles()
            self.remove_dead_particles()
            self.resample_if_required()
            self.render_particles()
    
    def get_laser_msg(self):
        msg = self.laser_queue.get()
        self.laser_queue.queue[:] = []
        return msg

    def get_robot_readings(self, lmsg):
        msg = self.laser_queue.get()
        r = msg.ranges
        return r

    def resample_if_required(self):
        if len(self.particles) < RESAMPLE_THRESHOLD:
            self.resample(TOTAL_PARTICLES - len(self.particles))

    def converged_loc(self, strategy="centroid"):
        if strategy == "centroid":
            coords = np.asarray([(p.x, p.y, p.theta) for p in self.particles])
            sum_x = np.sum(coords[:, 0])
            sum_y = np.sum(coords[:, 1])
            sum_z = np.sum(coords[:, 2])

            min_a = min([p.theta % 360 for p in self.particles])
            max_a = max([p.theta % 360 for p in self.particles])
            median_angle = np.median([p.theta for p in self.particles]) % 360

            print("min angle %0.2f, max angle %0.2f, med angle %0.2f"%(min_a, max_a, median_angle))

            centroid = sum_x / len(coords), sum_y / len(coords), sum_z / len(coords)
            particles_near_centroid = len([1 for p in self.particles if p.d3_get_distance_to(*centroid) < 1])

            if particles_near_centroid > CENTROID_THRESHOLD:
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

    #def sonar_update(self, smsg):
    #    self.sonar_queue.put(smsg)

def main():
    rospy.init_node("localize", anonymous=True)
    root = tk.Tk()
    initialize_cspace()
    #l = Localizer('/home/stu12/s11/mhs1841/catkin_ws/src/hw1/src/scripts/project.png', master=root, height=700, width=2000)
    # we are bad people
    l = Localizer('/home/stu9/s4/bwb5381/project.png', master=root, height=700, width=2000)
    rospy.Subscriber("/r1/kinect_laser/scan", LaserScan, l.laser_update)
    rospy.Subscriber("/r1/odom", Odometry, l.odom_update)
    t = Timer(0.1, l.update)
    t.start()
    root.mainloop()


if __name__ == "__main__":
    main()
