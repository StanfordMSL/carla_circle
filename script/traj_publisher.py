#!/usr/bin/env python

import rospy
from math import sqrt
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Vector3, Quaternion, Twist

def talker():
    rospy.init_node('traj_commander', anonymous=True)
    pub = rospy.Publisher('desired_waypoints', MultiDOFJointTrajectory, queue_size=10)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = MultiDOFJointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = '/map'

        pts = []
        num_pt = 30
        radius = 20
        ang_inc = 2 * math.pi / num_pt
        for i in range(num_pt):
            ang = ang_inc * i
            pt = MultiDOFJointTrajectoryPoint()
            pt.transforms = [Transform(Vector3(radius * cos(ang), radius * sin(ang)), \
                            Quaternion(0,0,0,1))]
            pt.velocities = [Twist(Vector3(5, 0, 0), Vector3(0,0,0))]
            pts.append(pt)
        msg.points = pts
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
