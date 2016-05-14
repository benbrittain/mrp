#!/usr/bin/env python
"""
manual.py
Drive it yourself.
"""

import rospy
import sys
import math
import curses
import argparse
from p2os_msgs.msg import MotorState, SonarArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance, Point, Quaternion, Twist

__author__ = "Zach Lauzon"

stdscr = curses.initscr()
curses.noecho()
curses.cbreak()
stdscr.nodelay(True)
stdscr.keypad(True)

INITIAL_SPEED = 0.3

def quaternion_to_z_rotation(q):
    """Converts a quaternion to a Z-rotation in radians."""
    return 2.0 * math.atan2(q.z, q.w)

def normalized_radians(a):
    """Translates an 'angular distance' into the smallest motion around a circle."""
    return (a + math.pi) % (2.0*math.pi) - math.pi

class Manual:

    def __init__(self, simulated=False, debug=False):
        """Configures Publishers and configuration."""
        rospy.init_node(self.__class__.__name__, anonymous=True)
        self.rate = rospy.Rate(10) # 10hz
        self.debug = debug
        ns = '/r1/' if simulated else '/'
        self.dlog("namespace: " + str(ns))
        self.linear_speed = INITIAL_SPEED

        self.motor_state_pub = rospy.Publisher(ns + 'cmd_motor_state', MotorState, latch=True, queue_size=10)
        self.vel_pub = rospy.Publisher(ns + 'cmd_vel', Twist, queue_size=1)
        self.motor_state_pub.publish(1)
        self.dlog("set motor state!")
        self.rate.sleep()

    def manual_control(self):
        """Navigates the robot to the provided point."""
        condition = True
        while condition and not rospy.is_shutdown():
            vel = Twist()
            vel.linear.y = 0.0
            vel.linear.x = 0.0
            vel.angular.z = 0.0

            try:
                key = stdscr.getkey()
                #rospy.loginfo(key)
                curses.flushinp()
                if key == "KEY_LEFT" or key == "a":
                    vel.angular.z = self.linear_speed
                elif key == "KEY_RIGHT" or key == "d":
                    vel.angular.z = -self.linear_speed
                elif key == "KEY_UP" or key == "w":
                    vel.linear.x = self.linear_speed
                elif key == "KEY_DOWN" or key == "s":
                    vel.linear.x = -self.linear_speed
                elif key == "q":
                    vel.linear.x = self.linear_speed
                    vel.angular.z = self.linear_speed
                elif key == "e":
                    vel.linear.x = self.linear_speed
                    vel.angular.z = -self.linear_speed
                elif key == "c":
                    vel.linear.x = -self.linear_speed
                    vel.angular.z = self.linear_speed
                elif key == "z":
                    vel.linear.x = -self.linear_speed
                    vel.angular.z = -self.linear_speed
                elif key == "=":
                    self.linear_speed += 0.1
                elif key == "-":
                    self.linear_speed = max(0.1, self.linear_speed - 0.1)
            except:
                pass

            #self.log("publishing {vel}".format(vel=vel))
            self.vel_pub.publish(vel)
            self.rate.sleep()

        stop_vel = Twist()
        stop_vel.angular.z = 0
        stop_vel.linear.x = 0
        self.vel_pub.publish(stop_vel)

    def run(self):
        self.log("run() started")
        self.manual_control()
        self.stop()

    def stop(self):
        """Stops the robot and locks up the motor state."""
        self.log("stop() called. Cleaning up...")
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        self.vel_pub.publish(vel)

        self.motor_state_pub.publish(0)

        curses.echo()
        curses.nocbreak()

    def log(self, msg, debug=False):
        """Logs messages to rospy depending on current configuration."""
        if debug == False or self.debug:
            rospy.loginfo(msg)

    def dlog(self, msg):
        """A debug convenience function."""
        self.log(msg, debug=True)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Manually drive around the Pioneer robot')
    parser.add_argument('-s', action='store_true', help='set this flag if running in the simulator')

    args = parser.parse_args()
    
    runner = Manual(simulated=args.s, debug=True)

    try:
        runner.run()
    except rospy.ROSInterruptException, KeyboardInterrupt:
        runner.stop()
