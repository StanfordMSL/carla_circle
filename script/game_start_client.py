#!/usr/bin/env python
# mingyuw@stanford.edu


import rospy
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from math import *

def restart_client():

    rospy.init_node('restart_client', anonymous=True)

    try:
        srv_name = 'MSLcar0/restart'
        rospy.wait_for_service(srv_name)
        restart = rospy.ServiceProxy(srv_name, Empty)
        restart()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == '__main__':
    try:
        restart_client()
    except rospy.ROSInterruptException:
        pass
