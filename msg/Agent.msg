# ---------------- /agent.msg -------------------
# Custom ROS message for vehicle stats information

Header header
geometry_msgs/Pose pose
geometry_msgs/Twist twist
float64 speed
carla_circle/PathArray future_pathArray
