# Stanford MSL (Carla) AV Stack

## Overview

This repository provides the MSL's AV Stack that works with the Carla Simulator.
The instructions below explain how to setup a Catkin workspace for ROS. At the
moment, this stack is designed for the Carla MCity Event 1.

[comment]: <> (
    TODO: Add details about the high and low level architecture of the AV stack
)

## Carla Version / Carla ROS Version

For this challenge we use special MCity binaries provided by Univ. of Michigan.
The binaries use Carla version 0.9.9.4 and have a special map designed for the
challenges.

## Setup the Catkin Workspace

Start by creating a new catkin workspace directory. Something like

```bash
source /opt/ros/melodic/setup.bash
mkdir -p ~/Catkin_Workspace/tri-carla-challenges/src
cd ~/Catkin_Workspace/tri-carla-challenges
```

We need to clone our repositories to the `src` directory

```bash
cd ~/Catkin_Workspace/tri-carla-challenges/src
git clone git@github.com:StanfordMSL/carla_circle.git -b update_planners
git clone https://github.com/carla-simulator/ros-bridge.git
cd ~/Catkin_Workspace/tri-carla-challenges/src/ros-bridge
git reset --hard 1096747a6e84bf01ea1a859b9a0f6d01e0304ef6
git submodule update --init
cd ../
rosdep update
rosdep install --from-paths src --ignore-src -r
```

Then we need to compile our Catkin workspace.

**The first time you run this you do not need to run the command
`catkin clean --yes`. If you do run the command, you will get an error, but this
can be ignored.**

```bash
cd ~/Catkin_Workspace/tri-carla-challenges
catkin clean --yes
catkin build carla_ackermann_control carla_circle carla_ego_vehicle carla_infrastructure carla_manual_control carla_msgs carla_ros_bridge carla_waypoint_publisher
catkin build rqt_carla_control
```

## Running the AV Client

***Note:**
We assume an instance of Carla is running and available on the localhost. By
default, running `./CarlaUE4.sh` from the Carla simulator directory should be
sufficient. However, if running on Michigan's server, you should follow the
instructions found [here](https://yyab.github.io/mcityCarlaChallenge.io/).*

We should now be able to run the AV client on the server. In a terminal enter
the following:

```bash
export PYTHONPATH=$PYTHONPATH:/opt/Carla/MCity/PythonAPI/carla/dist/carla-0.9.9-py2.7-linux-x86_64.egg
source ~/Catkin_Workspace/tri-carla-challenges/devel/setup.bash
roslaunch carla_circle mcity_event1.launch host:=localhost port:=2000 max_speed:=5.0 opp_vel:=5.0 plan_steps:=10 plan_horizon:=3.0
```

# Notes on the MCity specific challenge

Michigan is hosting the challenge on a server they run. The server ip is
`141.211.37.251`. To log in, we need to use the username `stanford`. As
mentioned above, you should check out their documentation to better
understand the process. As a quick refresher, you should have a terminal
in which you can explicitly run the command
`ssh stanford@141.211.37.251 -L 2000:localhost:2000 -L 2001:localhost:2001 -L 2002:localhost:2002`.
This will forward the 2000-2002 ports of your local machine to Michigan's
server where the Carla simulation is running. Before running on the
Michigan server, you should add a timeslot to the
[calendar](https://calendar.google.com/calendar/u/1?cid=Y18xMGMxNjcybmFmNXFzNG1jZnFmMWRrdjBmc0Bncm91cC5jYWxlbmRhci5nb29nbGUuY29t)
You may also want to monitor the simulation by viewing the stream at
https://www.twitch.tv/cmpute.

## Important Links

- Calendar - https://calendar.google.com/calendar/u/1?cid=Y18xMGMxNjcybmFmNXFzNG1jZnFmMWRrdjBmc0Bncm91cC5jYWxlbmRhci5nb29nbGUuY29t
- Challenge documentation - https://yyab.github.io/mcityCarlaChallenge.io/
- Carla server stream - https://www.twitch.tv/cmpute

You most likely need to request access to the calendar by emailing Xinpeng at `xinpengw at umich.edu`
