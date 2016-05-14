# mrp

# Installation
```
git clone https://github.com/cavedweller/mrp.git


If in the google.rit domain, This is the pre-computed cspace file.
https://drive.google.com/open?id=0B-9CT7zfTLB1UV9Rc3Q3VDZySG8

```

# Running
Initialize all subcomponents. Then publish the end goal to '/endgoal' as a std_msgs/String to tell
it where to move. Ex:

    rostopic pub '/endgoal' std_msgs/String ' -18.4 -8.7' -1

# Project Components
The components may be started in any order. They use the ROS message sub-system to communicate.

## initializer.py
The ROS Module that starts the localization. It's fairly simple, it does a 360 rotation, moves
forward, then repeats. It cancels after the first localization message has been published.

    rosrun localize initializer.py

## localizer.py
The ROS Module that does the localization. The default version up on github is using an extremely
conservative sensor model that functions in real space. This sensor model also worked in simulation
during testing. It uses a pickled pre-computed configuration space. This file must be at the root of
where you are launching the node. It publishes the localized position as fast as an update can
happen.

In simulation, succesful trials of full map localization were done using 200000 particles. Usually,
4200 are used.

    rosrun localize localizer.py

## planner.py
The ROS Module that does the path planning. It depends on no p2os primitives or channels. It simply
listens on the channels '/localized_pos' and '/endgoal' it publishes the position that the robot
should move to on '/goal'. It uses A\* path planning on a 25% scaled image as the search space.
There is a transition function that pushes away from walls to make a nice path. When it sees the
robot is near the next step from '/goal' (which shoulda been called step), it tells safegoto the
new location to move to.

    rosrun localize planner.py

## safegoto.py
The ROS Module that does the movement. Receives constant updates from localizer to correct for error
in the onboard motor encoders. Uses vector fields to dodge obstacles.

    rosrun localize safegoto.py




