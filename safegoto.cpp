// Ben Brittain

#include <sstream>
#include <string>
#include <fstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "p2os_msgs/MotorState.h"
#include "p2os_msgs/SonarArray.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"
#include <signal.h>
#include <math.h>
#include <vector>

#define PI 3.14159265

double distance(double x0, double y0, double x1, double y1) {
    return sqrt(pow(x0 - x1, 2) + pow(y0 - y1, 2));
}

class Vec {
    public:
        double x;
        double y;
        Vec(): x(0.0), y(0.0) {}
        Vec(const double& ix, const double& iy): x(ix), y(iy) {}

        Vec& operator+=(const Vec& v)  { x += v.x; y += v.y; return *this; }
        Vec& operator-=(const Vec& v)  { x -= v.x; y -= v.y; return *this; }
        Vec& operator*=(const Vec& v)  { x *= v.x; y *= v.y; return *this; }
        Vec& operator*=(const double& s) { x *= s; y *= s; return *this; }
};
Vec operator+(const Vec& lhs, const Vec& rhs) { 
    return Vec(lhs.x, lhs.y) += rhs;
}
Vec operator-(const Vec& lhs, const Vec& rhs) { 
    return Vec(lhs.x, lhs.y) -= rhs;
}
Vec operator*(const Vec& lhs, const Vec& rhs) { 
    return Vec(lhs.x, lhs.y) *= rhs;
}

double magnitude(const Vec &v) {
    return sqrt((v.x * v.x) + (v.y * v.y));
}

Vec normal(const Vec &v) {
    double mag = magnitude(v);
    return Vec(v.x/mag, v.y/mag);
}

// handle shutdown, stop the robot
void onShutdown(int sig) {
    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    geometry_msgs::Twist ts;
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 ang;
    linear.x = 0;
    linear.y = 0;
    linear.z = 0;
    ang.x = 0;
    ang.y = 0;
    ang.z = 0;
    ts.linear = linear;
    ts.angular = ang;
    vel_pub.publish(ts);
    ROS_INFO("Moriturus te saluto.");
    ros::shutdown();
}

class Tars {
    public:
        Tars(ros::NodeHandle& n);
        ~Tars();
        // methods to put in main loop
        void move();
        void updatePath();
        // HW dependent 
        void set_goal(double x, double y);
        bool goal_finished();
        bool has_goal();
        // ROS callbacks
        void updateLocation(const nav_msgs::Odometry::ConstPtr &odom);
        void updateLaser(const sensor_msgs::LaserScan::ConstPtr &laser);
        //void updateSonar(const p2os_msgs::SonarArray::ConstPtr &sonar);

    private:
        double x;
        double y;
        double theta;
        Vec robot_vec;
        ros::Publisher vel_pub;
        ros::Publisher motor_pub;

        double goal_x;
        double goal_y;
        Vec goal_vec;
        bool goal;

        double waypoint_x;
        double waypoint_y;

        // obstacle avoidance stuff
        std::vector<Vec> obstacles; 
        std::vector<Vec> dropped_repulsors; 
        int cluster_count;

};

Tars::Tars(ros::NodeHandle& n) { // n is internally reference counted
    ROS_INFO("Everybody good? Plenty of slaves for my robot colony?");
    vel_pub = n.advertise<geometry_msgs::Twist>("/r1/cmd_vel", 1000);
    motor_pub = n.advertise<p2os_msgs::MotorState>("/r1/cmd_motor_state", 1000, true);
    p2os_msgs::MotorState ms;
    ms.state = 1;
    motor_pub.publish(ms);
    cluster_count = 0;
}


void Tars::updatePath() {
    Vec attractor= normal(robot_vec - goal_vec);
    attractor *= -1.5; // scale it


    Vec repulsors;
    for (int i = 0; i < obstacles.size(); i++) {
        Vec obs_vec = obstacles[i];
        Vec repulsor;
        repulsor = normal(robot_vec - obs_vec);// * normal(robot_vec - obs_vec);
//        repulsor *= normal(robot_vec - obs_vec);
        //repulsor *= (0.05); // scale
        repulsor *= (0.075); // scale
        ROS_INFO("%f, %f", repulsor.x, repulsor.y);
        repulsors = repulsors + repulsor;
    }
//
//    if (cluster_count % 15 == 1) {
//        //ROS_INFO("Dropped Repulsor");
//        dropped_repulsors.push_back(Vec(robot_vec.x, robot_vec.y));
//    }
//    cluster_count++;

//    for (int i = 0; i < dropped_repulsors.size(); i++) {
//        Vec obs_vec = dropped_repulsors[i];
////        if (distance(robot_vec.x, robot_vec.y, obs_vec.x, obs_vec.y) < 1) {
//            Vec repulsor = normal(robot_vec - obs_vec);
//            repulsor *= (0.5); // scale
//            repulsors = repulsors + repulsor;
// //       }
//    }

    Vec vector_field = attractor + repulsors;
    Vec goal_vec = vector_field + robot_vec; // add vectors to current pos vector
    ROS_INFO("%f, %f", goal_vec.x, goal_vec.y);
    waypoint_x = goal_vec.x;
    waypoint_y = goal_vec.y;
}

void Tars::move() {
    geometry_msgs::Twist ts;
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 ang;

    // if no goal, don't do anything
    if (!has_goal()) {
        ang.z = 0;
        linear.x = 0;
        ts.linear = linear;
        ts.angular = ang;
        vel_pub.publish(ts);
        return;
    }

    // if not in boundry following mode, head toward goal
    theta = (fmod(theta + PI, 2*PI) - PI);
    double mline_theta = fmod(atan2(waypoint_y - y, waypoint_x - x) + PI, 2*PI) - PI;
    double rot = 0.2 * (fmod(mline_theta - theta + PI, 2*PI) - PI);
/* TODO: examine bringing this back
    if (rot > (PI/4.0)) {
        //ROS_INFO("mtheta: %f rtheta: %f rot : %f", mline_theta, theta, rot);
        ang.z = rot;
    } else {
*/
        ang.z = rot;
        linear.x = 0.1;// * sqrt(pow(x,2) + pow(y,2));
//  }
    ts.linear = linear;
    ts.angular = ang;
    vel_pub.publish(ts);
}

void Tars::set_goal(double ax, double ay) {
    goal_x = ax;
    goal_y = ay;
    goal_vec.x = ax;
    goal_vec.y = ay;
    goal = true;
    ROS_INFO("SETTING NEW TARGET: (%f, %f)", goal_x, goal_y);
}

bool Tars::goal_finished() {
    if (fabs(goal_x - x) < 0.1 && fabs(goal_y - y) < 0.1) {
        goal = false;
        return true;
    }
    return false;
}

bool Tars::has_goal() {
    return goal;
}

void Tars::updateLocation(const nav_msgs::Odometry::ConstPtr &odom) {
    tf::Pose pose;
    tf::poseMsgToTF(odom->pose.pose, pose);
    theta = fmod(tf::getYaw(pose.getRotation()), 2*PI);
    x = odom->pose.pose.position.x;
    y = odom->pose.pose.position.y;
    robot_vec.x = x;
    robot_vec.y = y;
}

void Tars::updateLaser(const sensor_msgs::LaserScan::ConstPtr &laser) {
    float prev_range = laser->ranges[0];
    obstacles.clear();
    double goal_theta = fmod(atan2(goal_y - y, goal_x - x), 2*PI);
    bool blocked = false;
    for (int i = 0; i < laser->ranges.size(); i++) {
        double range_theta = laser->angle_min + (i * laser->angle_increment);
        double range_dist = laser->ranges[i];
        double range_x = x + range_dist * cos(theta + range_theta);
        double range_y = y + range_dist * sin(theta + range_theta);
        if (range_dist < 0.8) {
            obstacles.push_back(Vec(range_x, range_y));
        }
    }
}

Tars::~Tars() {
    // yolo
}

double goal_a, goal_b; //globals, mah bad

void goalCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I recieved a new goal: [%s]", msg->data.c_str());
    std::istringstream iss(msg->data.c_str());
    double a, b;
    if (!(iss >> a >> b)) { ROS_INFO("INVALID LOCATION"); } // error
    goal_a = a;
    goal_b = b;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Tars", ros::init_options::NoSigintHandler);
    signal(SIGINT, onShutdown);

    // set up robot
    ros::NodeHandle n; 
    Tars robot(n);
    ros::Subscriber odom_sub = n.subscribe("/r1/odom", 1,
            &Tars::updateLocation, &robot);
    ros::Subscriber laser_sub = n.subscribe("/r1/kinect_laser/scan", 1, 
            &Tars::updateLaser, &robot);
    ros::Subscriber goal_sub = n.subscribe("/goal", 1, goalCallback);

    ros::Rate r(10);
    std::ifstream fin(argv[1]);
    std::string line;

    double last_a = -1.0;
    double last_b = -1.0;
    while (true) {
        if(last_a != goal_a || last_b != goal_b) {
            robot.set_goal(goal_a,goal_b);
            last_a = goal_a;
            last_b = goal_b;
        }
        if (robot.goal_finished() && (!robot.has_goal())) {
            ROS_INFO("Reached last goal");
        }

        //    std::getline(fin, line);
        //    std::istringstream iss(line);
        //    double a, b;
        //    if (!(iss >> a >> b)) { break; } // error
        //}
        robot.updatePath();
        robot.move();
        ros::spinOnce();
        r.sleep();
    }

    // TODO: robot.stop()
    onShutdown(0);
}
