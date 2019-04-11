#!/usr/bin/env python

import rospy
from math import sqrt
import math
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Vector3, Quaternion, Twist, Point
from visualization_msgs.msg import Marker

def talker():
    rospy.init_node('traj_commander', anonymous=True)
    pub = rospy.Publisher('desired_waypoints', MultiDOFJointTrajectory, queue_size=10)
    viz_pub = rospy.Publisher('viz/desired_waypoints', Marker, queue_size=10)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = MultiDOFJointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = '/map'

        viz_msg = Marker()
        viz_msg.header.stamp = rospy.Time.now()
        viz_msg.header.frame_id = '/map'
        viz_msg.type = Marker.CUBE_LIST
        viz_msg.scale.x = 1
        viz_msg.scale.y = 1
        viz_msg.scale.z = 1
        viz_msg.color.a = 1
        viz_msg.points = []

        pts = []
        num_pt = 30
        radius = 20
        ang_inc = 2.0 * math.pi / num_pt
        for i in range(num_pt):
            ang = ang_inc * i
            pt = MultiDOFJointTrajectoryPoint()
            pt.transforms = [Transform(Vector3(radius * math.cos(ang), radius * math.sin(ang), 0), \
                            Quaternion(0,0,0,1))]
            pt.velocities = [Twist(Vector3(5, 0, 0), Vector3(0,0,0))]
            viz_msg.points.append(Point(radius * math.cos(ang), radius * math.sin(ang), 0))
            pts.append(pt)
        msg.points = pts
        pub.publish(msg)
        viz_pub.publish(viz_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
