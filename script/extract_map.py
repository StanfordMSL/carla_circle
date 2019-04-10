#!/usr/bin/env python
import rospy
from std_msgs.msg import String



class Writer:

    def __init__(self):
        rospy.init_node('writer', anonymous=True)

        rospy.Subscriber('/carla/map', String, self.map_cb)

        self.recorded = False
      
        self.map_file = open("/home/mingyu/catkin_ws/src/ros-bridge/resource/map", "w")

    def map_cb(self, msg):
    	if not self.recorded:
	        self.map_file.write(msg.data)
	        self.recorded = True


if __name__ == '__main__':
    try:
        bri = Writer()
        rospy.spin()
    except rospy.ROSInterruptException:
		pass