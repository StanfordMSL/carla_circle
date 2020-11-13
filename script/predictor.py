#!/usr/bin/env python

# author: simonlc@stanford.edu

import rospy
import tf
from nav_msgs.msg import Odometry, Path, PoseStamped, Pose
from carla_circle.msg import PathArray
from geometry_msgs.msg import Transform, Vector3, Quaternion, Twist, Point

import numpy as np
import math
import timeit


class odom_state(object):
    def __init__(self, circle_center, circle_radius):
        self.time = None
        self.x = None
        self.y = None
        self.yaw = None
        self.vx = None
        self.vy = None
        self.speed = None

    def update_vehicle_state(self, odom_msg):
        '''
        input: ROS Odometry message
        output: None
        updates the time instance, position, orientation, velocity and speed
        '''
        self.time = odom_msg.header.stamp.to_sec()
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y
        self.z = odom_msg.pose.pose.position.z
        self.ori_quat = (odom_msg.pose.pose.orientation.x,
                    odom_msg.pose.pose.orientation.y,
                    odom_msg.pose.pose.orientation.z,
                    odom_msg.pose.pose.orientation.w)
        self.ori_euler = tf.transformations.euler_from_quaternion(self.ori_quat)
        self.yaw = self.ori_euler[2]
        self.vx = odom_msg.twist.twist.linear.x
        self.vy = odom_msg.twist.twist.linear.y
        self.speed = np.sqrt(self.vx**2 + self.vy**2)

    def get_position(self):
        return [self.x, self.y]

    def get_pose(self):
        return [self.x, self.y, self.yaw]

    def get_velocity(self):
        return [self.vs, self.vy]

    def get_speed(self):
        return self.speed

class TrajectoryPredictor:
    def __init__(self):
        rospy.init_node("predictor", anonymous=True)

        # retrieve ros parameters
        self.time_step = rospy.get_param("~time_step")
        self.horizon = rospy.get_param("~horizon")
        self.steps = int(self.horizon / self.time_step)

        # class attributes
        self.predicted_traj = Path()     # predicted trajectories for relevant cars
        # predicted _trajectories
        # each path in PathArray.paths has self.steps poses
        # the first pose start from the current location of the relevant car
        # the distance between sequential poses is speed(car) * self.time_step
        # this information is used to infer the possible time when the relevant cars
        # enter the roundabout

        self.state = odom_state()
        self.stateReady = False

        # subscribers, publishers
        rospy.Subscriber("/carla/vehicle/<ID>/odometry", Odometry, self.odom_cb) # need to deal with <ID>
        self.trajectory_pub = rospy.Publisher("/carla/vehicle/<ID>/predicted_traj", \ # need to deal with <ID>
                                Path, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)


    '''
    this function updates the ego vehicle state in class
    '''
    def odom_cb(self, msg):
        self.state.update_vehicle_state(msg)
        if not self.stateReady:
            self.stateReady = True

    def timer_cb(self, event):
        if self.stateReady:
            pos_x, pos_y = self.state.get_position()
            velocity = self.state.get_velocity()

            traj_msg = Path()
            traj_msg.header.stamp = rospy.Time.now()
            traj_msg.header.frame_id = 'map'
            traj_msg.poses = []
            for k in range(sefl.steps+1):
                # Fill position and velocity at time step k in the traj_msg
                dt = k*self.time_step
                xk = pos_x + dt*velocity[0]
                yk = pos_y + dt*velocity[1]

                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = traj_msg.header.stamp + dt # not sure the time units are the same here
                pose_stamped.header.frame_id = 'map'
                pose_stamped.pose = Pose(
                    Point(xk, yk, self.z),
                    sefl.ori_quat,
                    )

                traj_msg.poses.append(pose_stamped)

            self.trajectory_pub.publish(traj_msg)


if __name__ == '__main__':
    try:
        predictor = TrajectoryPredictor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass