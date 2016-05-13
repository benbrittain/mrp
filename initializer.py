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

from threading import Timer
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from p2os_msgs.msg import SonarArray
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String
from p2os_msgs.msg import MotorState
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist


from functools import partial
from multiprocessing import Pool

INITIAL_SPEED = 0.08

LOCAL_DIST_INIT_ROT = math.pi/16.0
LOCAL_DIST_FWD = 16.0
LOCAL_DIST_ROT = 2*math.pi

class Initializer(object):
    def __init__(self, simulated=False):
        """Configures Publishers and configuration."""
        rospy.init_node('initializer', anonymous=True)
        self.rate = rospy.Rate(10)
        #ns = '/r1/' if simulated else '/'
        ns = '/'
        self.linear_speed = INITIAL_SPEED
        self.motor_state_pub = rospy.Publisher(ns + 'cmd_motor_state', MotorState, latch=True, queue_size=10)
        self.vel_pub = rospy.Publisher(ns + 'cmd_vel', Twist, queue_size=10)
        self.motor_state_pub.publish(1)
        self.rate.sleep()

        self.last_theta = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        self.total_mov = 0.0
        self.total_rot = 0.0
        self.init_rot = 0.0
        self.odom_queue = Queue.LifoQueue()

        self.localized = False

    def attempt_localize(self):
        while not self.localized and not rospy.is_shutdown():
            vel = Twist()
            vel.linear.y = 0.0
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            (dx, dy, dtheta) = self.get_movement()
            print("%0.2f | %0.2f"%(self.total_mov, self.total_rot))
            dist = math.sqrt(dx**2 + dy**2)
            if self.total_rot < LOCAL_DIST_ROT:
                vel.angular.z = INITIAL_SPEED
                self.total_rot += abs(dtheta)
            else:
                if self.total_mov < LOCAL_DIST_FWD:
                    vel.linear.x = INITIAL_SPEED
                    self.total_mov += dist
                else:
                    # reset for more info gathering
                    self.total_mov = 0.0
                    self.total_rot = 0.0
            self.vel_pub.publish(vel)
            self.rate.sleep()

        stop_vel = Twist()
        stop_vel.angular.z = 0
        stop_vel.linear.x = 0
        self.vel_pub.publish(stop_vel)

    def odom_update(self, omsg):
        self.odom_queue.put(omsg)

    def get_movement(self):
        omsg = self.odom_queue.get()
        if omsg is None:
            return None
        self.odom_queue.queue[:] = []

        self.x = x = omsg.pose.pose.position.x
        self.y = y = omsg.pose.pose.position.y
        q = (omsg.pose.pose.orientation.x,
             omsg.pose.pose.orientation.y,
             omsg.pose.pose.orientation.z,
             omsg.pose.pose.orientation.w)
        self.theta = euler = tf.transformations.euler_from_quaternion(q)
        dtheta = (self.last_theta - euler[2])
        l = (self.last_theta + math.pi)%(2.0*math.pi) - math.pi
        e = (euler[2] + math.pi)%(2.0*math.pi) - math.pi
        dtheta = (l - e + math.pi)%(2.0*math.pi) - math.pi
        dx = self.last_x - x
        dy = self.last_y - y

        dist = math.sqrt(dx**2 + dy**2)
        dist = dist if dist > 0.2 else 0
        if dist > 0:
            self.last_x = x
            self.last_y = y
        if abs(dtheta) < 0.2:
            dtheta = 0
        else:
            self.last_theta = euler[2]

        return(dx, dy, dtheta)

    def run(self):
        self.attempt_localize()
        self.stop()
        
    def stop(self):
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        self.vel_pub.publish(vel)
        self.motor_state_pub.publish(0)

    def pos_update(self, msg):
        self.localized = True
        print("Localized! Stopping")
        self.stop()

def main():
    tars = Initializer()
    rospy.Subscriber("/pose", Odometry, tars.odom_update)
    rospy.Subscriber("/localized_pos", String, tars.pos_update)
    try:
        tars.run()
    except rospy.ROSInterruptException, KeyboardInterrupt:
        tars.stop()

if __name__ == "__main__":
    main()
