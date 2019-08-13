#!/usr/bin/env python

import rospy
from math import sqrt
import math
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Vector3, Quaternion, Twist, Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry

def talker():
    rospy.init_node('odom_talker', anonymous=True)
    ego_pub = rospy.Publisher('/MSLcar0/ground_truth/odometry', Odometry, queue_size=10)
    ado_pub = rospy.Publisher('/MSLcar1/ground_truth/odometry', Odometry, queue_size=10)


    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        ego_msg = Odometry()
        ego_msg.header.stamp = rospy.Time.now()
        ego_msg.header.frame_id = "map"
        ego_msg.pose.pose.position.x = 0
        ego_msg.pose.pose.position.y = 0
        ego_pub.publish(ego_msg)


        ado_msg = Odometry()
        ado_msg.header.stamp = rospy.Time.now()
        ado_msg.header.frame_id = "map"
        ado_msg.pose.pose.position.x = 0
        ado_msg.pose.pose.position.y = 0
        ado_pub.publish(ado_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
