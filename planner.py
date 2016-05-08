#!/usr/bin/python

import Tkinter as tk
from PIL import Image
import ImageTk

import colorsys
import heapq
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
from std_msgs.msg import String

from Utils import gcs2map, map2gcs, PriorityQueue

MAPWIDTH = 2000
MAPHEIGHT = 700
PIXELSIZE = 0.06346
SCALE = 0.25

def map2scale(x, y):
    return(int(SCALE * x), int(SCALE * y))

def scale2map(x, y):
    return((x / SCALE, y / SCALE))

def manhattan(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

# expensive but biases toward the center of hallways
# range is 1 (nothing) - 4
def wall_dist_cost(themap, (x, y)):
    for r in range(1, 4):
        adj = [(x+r, y+r), (x-r, y-r), (x-r, y+r), (x+r, y-r)]
        for (ax, ay) in adj:
            if themap[ax, ay] != (255, 255, 255):
                return (5 - r)
    return 1

def a_star(themap, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()
        if current == goal:
            break
        
        for n in neighbors(themap, current):
            new_cost = cost_so_far[current] + wall_dist_cost(themap, current)
            if n not in cost_so_far or new_cost < cost_so_far[n]:
                cost_so_far[n] = new_cost
                priority = new_cost + manhattan(goal, n)
                frontier.put(n, priority)
                came_from[n] = current

    return came_from, cost_so_far

def neighbors(themap, (x, y)):
    adj = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
    adj = filter(lambda (x, y): x > 0 and x < MAPWIDTH, adj)
    adj = filter(lambda (x, y): y > 0 and y < MAPHEIGHT, adj)
    adj = filter(lambda (x, y): (themap[x,y] == (255, 255, 255)), adj)
    return adj

class Planner(tk.Frame):    
    def __init__(self, *args, **kwargs):
        tk.Frame.__init__(self, *args, **kwargs)
        self.master.title("planner")
        self.master.minsize(width=MAPWIDTH,height=MAPHEIGHT)
        self.canvas = tk.Canvas(self,width=MAPWIDTH, height=MAPHEIGHT)
        self.canvas.pack()

        self.themap = Image.open("/home/stu9/s4/bwb5381/project.png")
        self.themap = self.themap.convert("RGB")
        self.mappix = self.themap.load()

        self.scalemap = self.themap.resize(map(lambda x: int(SCALE * x), 
                    self.themap.size), Image.BICUBIC)
        self.scalemappix = self.scalemap.load()

        self.mapimage = ImageTk.PhotoImage(self.themap)
        self.canvas.create_image(MAPWIDTH/2, MAPHEIGHT/2, image = self.mapimage)
        self.pack()
        self.step = (0,0)

        # localized coordinates queue
        self.loc_queue = Queue.LifoQueue()
        self.guessed_loc_queue = Queue.LifoQueue()
        # goal coordinates
        self.goal_queue = Queue.LifoQueue()
        self.goal_queue.put("-18.0 -9.0")

        self.draw_path = [] #TODO no reason for two, clean up
        self.path = []

        self.curr_loc = None
        self.goal_loc = None

        # rospy stuff
        self.goal_pub = rospy.Publisher('/goal', String, queue_size=1)


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

    def plan_path(self):
        if self.goal_loc == None:
            print("No Goal Location set!")
        elif self.curr_loc == None:
            print("No Known Location!")
        else:
            print("Planning new path")
            self.draw_path = [] #clear
            self.path = [] #clear

            map_curr_loc = map2scale(*gcs2map(*self.curr_loc))
            map_goal_loc = map2scale(*gcs2map(*self.goal_loc))
            came_from, cost_so_far = a_star(self.scalemappix, map_curr_loc, map_goal_loc)
            current = map_goal_loc
            path = [current]
            while current != map_curr_loc:
                try:
                    current = came_from[current]
                    path.append(current)
                except KeyError:
                    print("wat. ", current)
                    break
            path.reverse()
            self.draw_path = path
            self.draw_path = map(lambda x: scale2map(*x), path)
            self.path = map(lambda x: map2gcs(*scale2map(*x)), path)

    def update_image(self):
        # draw the path on the map
        if self.goal_loc != None:
            (goal_x, goal_y) = gcs2map(*self.goal_loc)
            for add_x in range(-3, 3):
                for add_y in range(-3, 3):
                    self.mappix[goal_x + add_x, goal_y + add_y] = (255, 0, 0)

        if self.curr_loc != None:
            (curr_x, curr_y) = gcs2map(*self.curr_loc)
            for add_x in range(-3, 3):
                for add_y in range(-3, 3):
                    self.mappix[curr_x + add_x, curr_y + add_y] = (0, 255, 0)

        for (step_x, step_y) in self.draw_path:
            for add_x in range(-1, 1):
                for add_y in range(-1, 1):
                    self.mappix[step_x + add_x, step_y + add_y] = (0, 0, 255)

        self.mapimage = ImageTk.PhotoImage(self.themap)
        self.canvas.create_image(MAPWIDTH/2, MAPHEIGHT/2, image = self.mapimage)
        self.themap = Image.open("/home/stu9/s4/bwb5381/project.png")
        self.themap = self.themap.convert("RGB")
        self.mappix = self.themap.load()

    def publish_next_step(self):
        if len(self.path) > 0:
            if len(self.path) > 15:
                self.step = self.path[14]
            else:
                self.step = self.path[-1]
            goal_str = "%f %f" %self.step
            print("going to %f %f"%self.step)
            self.goal_pub.publish(goal_str)

    def update(self):
        while True:
            trigger_plan = False
            loc_msg = self.get_queue(self.loc_queue)
            goal_msg = self.get_queue(self.goal_queue)

            guessed_loc_msg = self.get_queue(self.guessed_loc_queue)
            if guessed_loc_msg != None:
                guessed_loc = tuple(map(float, guessed_loc_msg.split(" ")))[:2]
                dist_to_step = manhattan(guessed_loc, self.step)
                if dist_to_step < 1:
                    print(dist_to_step)
                    self.curr_loc = guessed_loc
                    trigger_plan = True

            if loc_msg != None:
                self.curr_loc = tuple(map(float, loc_msg.split(" ")))[:2]
                trigger_plan = True
            if goal_msg != None:
                self.goal_loc = tuple(map(float, goal_msg.split(" ")))[:2]
                trigger_plan = True

            if trigger_plan:
                self.plan_path()
                self.publish_next_step()
                self.after(0, self.update_image)

    def goal_update(self, goal_msg):
        self.goal_queue.put(goal_msg.data)

    def loc_update(self, loc_msg):
        self.loc_queue.put(loc_msg.data)

    def guessed_loc_update(self, loc_msg):
        self.guessed_loc_queue.put(loc_msg.data)

def main():
    rospy.init_node("planner", anonymous=True)
    root = tk.Tk()
    planner = Planner(master=root, height=MAPHEIGHT, width=MAPWIDTH)
    rospy.Subscriber("/endgoal", String, planner.goal_update)
    rospy.Subscriber("/localized_pos", String, planner.loc_update)
    rospy.Subscriber("/guessed_pos", String, planner.guessed_loc_update)
    
    t = Timer(0.1, planner.update)
    t.start()
    root.mainloop()

if __name__ == "__main__":
    main()
