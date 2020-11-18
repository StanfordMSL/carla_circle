# carla_circle

Workspace for ROS and scripts to work with Carla MCity Event 1.

## carla version / carla ros version

For this challenge we use a special MCity binaries provided by Michigan. The
binaries use Carla version 0.9.9.4 and have a special map designed for the
challenges.


## Setup your Catkin workspace

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
cd ~/Catkin_Workspace/tri-carla-challenges/src
git clone git@github.com:StanfordMSL/uav_game.git -b carla
```

Then we need to compile our Catkin workspace.

**The first time you run this you do not need to run the command
`catkin clean --yes`. If you do run the command, you will get an error, which
can be ignored.**

```bash
cd ~/Catkin_Workspace/tri-carla-challenges
catkin clean --yes
export GUROBI_HOME=/opt/gurobi811/linux64
catkin build uavgame
catkin build carla_ackermann_control carla_circle carla_ego_vehicle carla_infrastructure carla_manual_control carla_msgs carla_ros_bridge carla_waypoint_publisher
catkin build rqt_carla_control
```

## Run the AV client

We should now be able to run the AV client on the server. In a terminal enter
the following:

```bash
export PYTHONPATH=$PYTHONPATH:/opt/Carla/MCity/PythonAPI/carla/dist/carla-0.9.9-py2.7-linux-x86_64.egg
source ~/Catkin_Workspace/tri-carla-challenges/devel/setup.bash
roslaunch carla_circle mcity_event1.launch host:=localhost port:=2000 max_speed:=5.0 opp_vel:=5.0 plan_steps:=10 plan_horizon:=3.0
```
