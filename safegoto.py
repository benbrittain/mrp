#!/usr/bin/env python
import sys
import time
import rospy
import math
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist


from sensor_msgs.msg import LaserScan


from p2os_msgs.msg import SonarArray
from p2os_msgs.msg import MotorState
from std_msgs.msg import String

import numpy as np
from math import atan2

# safegoto.py
#
# Program to make a robot go to a sequence of specified (x,y) locations.
# Avoiding obstacles with the help of LASER and SONAR sensor data, using 
# Artificial Potential Field Algorithm.
# Locations are passed in as a text file (first command line argument). 
#
# Usage: Usage:  rosrun hw3_pkg safegoto.py <location_file> <run_mode>
# 
# Author: Anurag Gomsale, RIT, Feb. 2016

########################################
# Class to define Navigator Object
########################################
class Navigator(object):
    def __init__(self, run_mode = 'sim'):
        rospy.init_node('navigator', anonymous=True)
        self.motor_publisher = rospy.Publisher("cmd_motor_state", MotorState,queue_size=10,latch = True)
        self.motor_publisher.publish(MotorState(1))
        
        self.loc_pub = rospy.Publisher('/guessed_pos', String, queue_size=1)
        
        self.last_planned_goal = None
        self.planned_goal = None
        
        self.is_new_localizer_pos = False
        self.loz_current_location = [0.0,0.0]
        self.loz_yaw = 0.0
        
        if run_mode == 'hw':
            print('In hardware mode')
            rospy.Subscriber('/pose', Odometry, self.update_odometer)
            rospy.Subscriber('/scan', LaserScan, self.update_laser_scanner)
            rospy.Subscriber('/sonar', SonarArray, self.update_sonar_scanner)
            self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10, latch = True)
            
            #### Final project code
            rospy.Subscriber("/goal", String, self.update_goal)
            rospy.Subscriber("/localized_pos", String, self.update_localized_pose)
        elif run_mode == 'sim':
            print('In simulator mode')
            rospy.Subscriber('/r1/odom', Odometry, self.update_odometer)
            rospy.Subscriber('/r1/kinect_laser/scan', LaserScan, self.update_laser_scanner)
            rospy.Subscriber('/r1/sonar', SonarArray, self.update_sonar_scanner)
            
            self.velocity_publisher = rospy.Publisher('r1/cmd_vel', Twist, queue_size=10, latch = True)
            
            #### Final project code
            rospy.Subscriber("/goal", String, self.update_goal)
            rospy.Subscriber("/localized_pos", String, self.update_localized_pose)
        else:
            print('Invalid Run Mode')
            
        # let the subscriber tap in to the odom topic    
        time.sleep(4)
        
    ###################################################
    # Method to get goal information from path planner
    ###################################################  
    def update_goal(self, goal_msg):
        if goal_msg is None:
            return
        
        coordinates = goal_msg.data.split()
        
        self.planned_goal = [float(coordinates[0]),float(coordinates[1])]
    
    ###################################################
    # Method to get goal information from localizer
    ###################################################  
    def update_localized_pose(self, localized_pose_msg):
        if localized_pose_msg == None:
            print("No pose from localizer")
            return
        
        pose = localized_pose_msg.data.split()
        self.loz_current_location = np.array([float(pose[0]),float(pose[1])]) 
        self.loz_yaw = float(pose[2]) - self.yaw
        self.is_new_localizer_pos = True
        
        
    ###########################################
    # Method to load location data from file
    ###########################################
    def load_locations(self):
        print("Loading location sequence: " + self.location_file) 
        locations = [self.location_file]
        '''
        loc_file = open(self.location_file)
        for location in loc_file:
            coordinate = location.split()
            
            if len(coordinate) == 0:
                continue
            
            x = float(coordinate[0])
            y = float(coordinate[1])
            locations.append([x,y])
        '''
        print("Location sequence to go to is loaded.")
        self.locations_to_go = locations
        

    #######################################################################
    # Call back method
    # Updates Navigator object with odometry messages from /odom topic
    ######################################################################
    def update_odometer(self, odometry_msg):
        self.odometry_msg =odometry_msg
        loc = odometry_msg.pose.pose.position
        
        #self.current_location = [loc.x,loc.y]
        #self.current_location = np.array([loc.x,loc.y])+np.array(self.loz_current_location)
        
        rotate_x = loc.x * np.cos(self.loz_yaw) - loc.y * np.sin(self.loz_yaw) + self.loz_current_location[0]
        rotate_y = loc.x * np.sin(self.loz_yaw) + loc.y * np.cos(self.loz_yaw) + self.loz_current_location[1]
        #x sin theta + y cos theta
        
        self.current_location = np.array([rotate_x,rotate_y])
        self.current_velocity = odometry_msg.twist.twist.linear.x
        
        q = odometry_msg.pose.pose.orientation
        
        # get robot's orientation w.r.t. z axis
        yaw = 2*math.atan2(q.z,q.w) + self.loz_yaw
        # maps it to [0,2*pi] 
        if yaw < 0:
            yaw += 2.0*math.pi
        #self.yaw = yaw 
        self.yaw = yaw 
        
        
    #######################################################################
    # Call back method
    # Updates Navigator object with LASER scan messages from /scan topic
    #######################################################################
    def update_laser_scanner(self, laser_scan_msg):
        if laser_scan_msg is None:
            return

        self.laser_scan_msg =laser_scan_msg
    
        self.min_laser_angle = laser_scan_msg.angle_min
        self.laser_res = laser_scan_msg.angle_increment
        self.min_laser_range = laser_scan_msg.range_min
        #self.max_laser_range = laser_scan_msg.range_max
        
        # limit robot's field of view to 0.8 m, works better in cramped 
        # environment such as hallway
        self.max_laser_range = 0.8
        #self.max_laser_range = 10.0
        
        
        ranges = np.array(self.laser_scan_msg.ranges)
        
        laser_angles = np.arange(0,ranges.size) 
        
        # filter the LASER data based on range values
        laser_ranges_filter = ([ (self.min_laser_range <= ranges) & (ranges < self.max_laser_range)])
        ranges =ranges[laser_ranges_filter]
        # add angle information for each LASER range reading  
        laser_angles = (laser_angles*self.laser_res)+self.min_laser_angle
        laser_angles = laser_angles[laser_ranges_filter]
        
        self.laser_ranges = np.stack((ranges,laser_angles))
        
    
    
    #######################################################################
    # Call back method
    # Updates Navigator object with SOANAR scan messages from /sonar topic
    #######################################################################
    def update_sonar_scanner(self, sonar_scan_msg):
        self.sonar_scan_msg =sonar_scan_msg
        sonar_ranges = np.array(sonar_scan_msg.ranges)
        # flip SONAR array to align with LASER data
        sonar_ranges = np.fliplr([sonar_ranges])[0]
       
         # limit robot's field of view to 0.8 m, works better in cramped 
        # environment such as hallway
        self.max_sonar_range = 0.8 #0.3      
        
        # add angle information for each SONAR range reading     
        sonar_angles = np.array([-90,-50,-30,-10,10,30,50,90])*np.pi/180
        
        # filter the SONAR data based on range values and angle. Filter the
        # noisy SONAR data for the range of angles already covered by the LASER
        sonar_final_filter = ([(sonar_ranges < self.max_sonar_range) & (sonar_ranges > 0.0) & (np.fabs(sonar_angles) >= np.pi/6.0)])
        
        sonar_angles = sonar_angles[sonar_final_filter]
        sonar_ranges = sonar_ranges[sonar_final_filter]
        
        self.sonar_ranges = np.vstack((sonar_ranges,sonar_angles))
       
    
    ##########################################################################
    # Method to calculate resultant repulsive potential vector from SONAR and
    # LASER field vectors
    ##########################################################################    
    def calculate_obstacle_potential_field(self):
        
        k_o_laser = 10
        k_o_sonar = 40
     
        laser_range_data = self.laser_ranges
        sonar_range_data = self.sonar_ranges
        
        laser_obstacle_vector = self.get_potential_field(laser_range_data,10)
        sonar_obstacle_vector = self.get_potential_field(sonar_range_data,40)
        return laser_obstacle_vector + sonar_obstacle_vector
    
    
    ###################################################################
    # Method to calculate repulsive potential vector for a given sensor
    ###################################################################
    def get_potential_field(self,sensor_range_data, k):
        
        yaw = self.yaw
        #yaw = self.loz_yaw 
        so_many_rows = sensor_range_data.shape[1]
        raw_data = np.column_stack((sensor_range_data.T, 
                                    np.zeros(so_many_rows),np.zeros(so_many_rows)))
        # add current orientation of the robot to the range data from sensor to
        # align it with respect to global map
        raw_data[:,1] = raw_data[:,1] + yaw
        # repulsive unit vector in x direction
        raw_data[:,2] = -np.cos(raw_data[:,1])/np.power(raw_data[:,0],2)
        # repulsive unit vector in y direction
        raw_data[:,3] = -np.sin(raw_data[:,1])/np.power(raw_data[:,0],2)
        
        return k*raw_data[:,2:4].sum(axis = 0)
    
    #############################################################
    # Method to calculate attractive potential vector toward goal
    #############################################################    
    def calculate_goal_potential_field(self):
        k_g = 250
        
        next_goal = self.next_goal
        current_loc = self.current_location
        #current_loc = self.loz_current_location
        d = self.get_euclidian_distance(next_goal, current_loc)
        
        next_goal = np.array(next_goal)
        current_loc = np.array(current_loc)
        goal_vector = k_g*(next_goal - current_loc)/d 
     
        return goal_vector
    
    ###########################################################################
    # Method to calculate difference between current robot orientation and 
    # required orientation considering the obstacles in the way.  
    ###########################################################################    
    def get_dYaw(self):
        
        yaw = self.yaw
        #yaw = self.loz_yaw 
        # get repulsive obstacle field vector
        o_bar = self.calculate_obstacle_potential_field()
        # get attractive goal field vector
        g_bar = self.calculate_goal_potential_field()
        result = o_bar + g_bar
        
        # Get the direction of resultant field vector
        required_orientetion =  math.atan2(result[1],result[0])
        
        # Map the slop to same range [0, 2*pi] as robot's orientation
        if required_orientetion <0:
            required_orientetion += 2.0*math.pi
        
        dYaw = required_orientetion - yaw
        
        # Find the optimum rotation required to achieve desired orientation 
        if math.fabs(dYaw) > math.pi:
            if dYaw > 0:
                dYaw-=2.0*math.pi 
            else:
                dYaw+=2.0*math.pi
        
        self.dYaw = dYaw      
        return dYaw    
    
     
    ###################################################
    # Get angular velocity as a function of delta angle 
    ###################################################
    def give_angular_velocity(self,dYaw):
        direction  = 1
        
        if dYaw < 0:
            direction  = -1
            
        if math.fabs(dYaw) >0.8:
            w_z = direction*0.4
        else:
            w_z = dYaw/1.5 #2.2
            
        return Vector3(0.0,0.0,w_z)
    
    ##########################################################################
    # Method to find distance between current robot location and goal location 
    ##########################################################################
    def get_delta_distance(self):
        
        msg = self.odometry_msg 
        pos = msg.pose.pose.position
        
        x = pos.x
        y = pos.y
        #x =  self.loz_current_location[0]
        #y =  self.loz_current_location[1]
        
        return self.get_euclidian_distance(self.next_goal,[x,y])
    
    
    #########################################################################
    # Get linear velocity as function of delta distance and obstacle distance  
    #########################################################################
    def give_linear_velocity(self,delta_dist):
           
        if delta_dist < 2: #delta_dist < 0.5:
             x_vel = delta_dist/2.3 #x_vel = delta_dist/3.0
            
        if delta_dist > 2:
             x_vel =  0.5
        
        #if self.laser_ranges.size != 0 or self.sonar_ranges.size != 0:
        #    x_vel = 0.1
             
        return Vector3(x_vel,0.0,0.0)
           
    #######################################################
    # Method to find euclidean distance between two points  
    ####################################################### 
    def get_euclidian_distance(self,point1,point2):
        return math.pow((math.pow(point1[0]-point2[0],2)+
                         math.pow(point1[1]-point2[1],2)),0.5) 
        
     
     
    #######################################################
    # Method to navigate the robot through given points  
    #######################################################          
    def navigate(self):
        so_many_sec = 0.0
        print ("Started navigating\n")
        
        while not self.is_new_localizer_pos:
            print("Waiting for localizer")
            wait = True
            
        while self.planned_goal is None:
            print("Waiting for path planner")
            wait = True
       
        time.sleep(3)
        self.next_goal = self.planned_goal
        
        dYaw = self.get_dYaw()
        delta_dist = self.get_delta_distance()
         
        start_time = rospy.get_time()
        publish_rate = rospy.Rate(10) 
        
        while True: # position accurate within  20 cm
            self.next_goal = self.planned_goal
            loc_str = "%f %f" %tuple(self.current_location)
            self.loc_pub.publish(loc_str)
            
            print("Current Loc; ",self.current_location,"Current yaw: ",self.yaw)
            print("Go to; ", self.next_goal)
            #time.sleep(0.2)
            linear_velocity_vector = self.give_linear_velocity(delta_dist)
            angular_velocity_vector = self.give_angular_velocity(dYaw)
    
            # stop angular velocity when delta-yaw is accurate within 0.01 rad
            if math.fabs(dYaw) < 0.01:
                angular_velocity_vector = Vector3(0.0,0.0,0.0)
            
            # give little linear velocity when delta-yaw is greater than 0.3 rad
            # this ensures that robot keeps moving forward when obstructed, in bug like 
            # fashion without using any memory. Inefficient but works.
            if  math.fabs(dYaw) > 0.3: #0.3
                linear_velocity_vector = Vector3(0.01,0.0,0.0)
            
            twist = Twist( linear_velocity_vector,angular_velocity_vector)
            self.velocity_publisher.publish(twist)
            
            dYaw = self.get_dYaw()
            delta_dist = self.get_delta_distance()
            
            publish_rate.sleep()
             
###############
# main program
###############
if __name__ == '__main__':
    # validate input arguments
    if len(sys.argv) !=1:
        print "Usage:  rosrun abg_pkg goto.py <location_file> <run_mode: sim / hw>"
    else:    
        try:
            gemini = Navigator()
            time.sleep(2)
            gemini.navigate()
     
        except rospy.ROSInterruptException:
            pass
