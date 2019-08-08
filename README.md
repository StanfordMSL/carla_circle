# carla_circle


workspace for ros and scripts to work with Carla





### carla version / carla ros version
0.9.2



### run simulation
follow the instructions on https://github.com/carla-simulator/ros-bridge/tree/0.9.2 to install required packages

##### in CARLA folder
./CarlaUE4.sh  -carla-server -windowed -ResX=320 -ResY=240   __to start a carla server __


##### in ros carla_circle folder
python spawn_npc.modified.py
python manual_control.py



##### to run simulation
roslaunch carla_ros_bridge client_with_rviz.launch     __for bridge and visualization__

roslaunch carla_circle main_sim.launch    __for control planning layers __
