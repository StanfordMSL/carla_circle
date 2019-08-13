#!/usr/bin/env python

# map_updater.py
# this node publishes the drivable track center and track with around a circle given current position of a car
# updates at a certain frequency
# this file is to work with optimization based game-theoretic planner
# author: mingyuw@stanford.edu

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry


import numpy as np


class odom_state(object):
    def __init__(self):
        self.time = None
        self.x = None
        self.y = None
        self.yaw = None
        self.vx = None
        self.vy = None
        self.speed = None

    def update_vehicle_state(self, odom_msg):
        self.time = odom_msg.header.stamp.to_sec()
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y
        ori_quat = (odom_msg.pose.pose.orientation.x,
                    odom_msg.pose.pose.orientation.y,
                    odom_msg.pose.pose.orientation.z,
                    odom_msg.pose.pose.orientation.w)
        ori_euler = tf.transformations.euler_from_quaternion(ori_quat)
        self.yaw = ori_euler[2]
        self.vx = odom_msg.twist.twist.linear.x
        self.vy = odom_msg.twist.twist.linear.y
        self.speed = np.sqrt(self.vx**2 + self.vy**2)
        # print("this is the current speed", self.speed)

    def get_position(self):
        return [self.x, self.y]

    def get_pose(self):
        return [self.x, self.y, self.yaw]

    def get_velocity(self):
        return [self.vs, self.vy]

    def get_speed(self):
        return self.speed


class MapUpdater:
    def __init__(self):
        rospy.init_node("map_updater", anonymous=True)

        # retrieve ros parameters
        self.steps = rospy.get_param("~steps")      # # of waypoints in the track
        self.distance = rospy.get_param("~distance")     # distance between two waypoints
        freq = rospy.get_param("~update_frequency")

        # state information
        self.stateReady = False
        self.state = odom_state()
        self.ado_stateReady = False
        self.ado_state = odom_state()

        # path information
        self.ego_track_info = Path()
        self.ado_track_info = Path()

        # subscribers, publishers
        rospy.Subscriber("/MSLcar0/ground_truth/odometry", Odometry, self.odom_cb)
        rospy.Subscriber("/MSLcar1/ground_truth/odometry", Odometry, self.ado_odom_cb)
        self.ego_track_pub = rospy.Publisher("/MSLcar0/mpc/ego_track_information", Path, queue_size=10)
        self.ado_track_pub = rospy.Publisher("/MSLcar0/mpc/ado_track_information", Path, queue_size=10)
        self.update_timer = rospy.Timer(rospy.Duration(1.0/freq), self.timer_cb)


    def odom_cb(self, msg):
        self.state.update_vehicle_state(msg)
        if not self.stateReady:
            self.stateReady = True


    def ado_odom_cb(self, msg):
        self.ado_state.update_vehicle_state(msg)
        if not self.ado_stateReady:
            self.ado_stateReady = True

    def get_path_from_position(self, position_x, position_y):

        track_center = np.zeros((2, self.steps))
        track_width = 3.5
        if abs(pos_x) < 25 and abs(pos_y) < 25:
            pos_the = np.arctan2(position_y, position_x)
            the = np.linspace(pos_the-0.5, pos_the - 0.5 + np.pi, self.steps)

            radius = 21.75
            for i in range(self.steps):
                track_center[:,i] = [radius*np.cos(the[i]), radius*np.sin(the[i])]

            return track_center, track_width
        elif pos_x > 24.9:
            # first generate path list
            for i in range(self.steps)
        elif pos_x < -24.9:
            pass
        elif pos_y < -24.9:
            pass
        else:
            # should be pos_y > 24.9

    def update_ego_track(self):
        if not self.stateReady:
            return
        ego_pos = self.state.get_position()
        track_c, track_w = self.get_path_from_position(ego_pos[0], ego_pos[1])
        self.ego_track_info.header.stamp = rospy.Time.now()
        self.ego_track_info.header.frame_id = "map"
        self.ego_track_info.poses = []
        for i in range(self.steps):
            pose_s = PoseStamped()
            pose_s.header = self.ego_track_info.header
            pose_s.pose.position.x = track_c[0,i]
            pose_s.pose.position.y = track_c[1,i]
            self.ego_track_info.poses.append(pose_s)
        # append the track width info as the last element of path.poses
        track_w_pose = PoseStamped()
        track_w_pose.header = self.ego_track_info.header
        track_w_pose.pose.position.x = track_w
        self.ego_track_info.poses.append(track_w_pose)


    def update_ado_track(self):
        ado_pos = self.ado_state.get_position()
        track_c, track_w = self.get_path_from_position(ado_pos[0], ado_pos[1])
        self.ado_track_info.header.stamp = rospy.Time.now()
        self.ado_track_info.header.frame_id = "map"
        self.ado_track_info.poses = []
        for i in range(self.steps):
            pose_s = PoseStamped()
            pose_s.header = self.ado_track_info.header
            pose_s.pose.position.x = track_c[0,i]
            pose_s.pose.position.y = track_c[1,i]
            self.ado_track_info.poses.append(pose_s)
        # append the track width info as the last element of path.poses
        track_w_pose = PoseStamped()
        track_w_pose.header = self.ado_track_info.header
        track_w_pose.pose.position.x = track_w
        self.ado_track_info.poses.append(track_w_pose)


    def timer_cb(self, event):

        # lazy update of track information
        if self.stateReady and self.ado_stateReady:
            self.update_ego_track()
            self.update_ado_track()

            self.ego_track_pub.publish(self.ego_track_info)
            self.ado_track_pub.publish(self.ado_track_info)

if __name__ == '__main__':
    try:
        supervisor = MapUpdater()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
