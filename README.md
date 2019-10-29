# carla_circle


workspace for ros and scripts to work with Carla Event 4



### carla version / carla ros version
0.9.5




### to run simulation  
./CarlaUE4.sh  -carla-server -windowed -ResX=640 -ResY=480      
python3 manual_control.py __from carla_circle package to launch ego_vehicle__  
python3 Event4.py     __from TRI-Carla-Challenges folder__  

roslaunch carla_ros_bridge carla_ros_bridge.launch    __for ros bridge__  


roslaunch carla_circle main_game.launch file __for main control and visualization__  
roslaunch uavgame main_mpc_gurobi.launch    __for trajectory planner__  
(still need rosrun uavgame osprey_start.py to kick off the simulation  
- update: relocated to main_game.launch in carla_circle package to call the restart service)
roslaunch carla_ackermann_control  carla_ackermann_control.launch      __translate ackermann message to lower level control__  
