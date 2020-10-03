#!/usr/bin/env python

# map_updater.py
# this node publishes the drivable track center and track with around a circle
# given current position of a car
# updates at a certain frequency
# this file is to work with optimization based game-theoretic planner
# author: mingyuw@stanford.edu

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path, Odometry


import numpy as np
from numpy import genfromtxt
from scipy.spatial import KDTree

import os
import random

import carla
from carla_circle.srv import GetAvailablePath
# from utilities.parsers import get_mcity_route


def get_mcity_route(filename):
    resource_folder = os.path.join(os.path.dirname(__file__), "../resource/")
    route_file = os.path.join(resource_folder, filename)
    route = None

    try:
        rospy.loginfo("Reading from file {}".format(route_file))
        data = genfromtxt(route_file, delimiter=',', skip_header=0)
        waypoints = []
        prev = [-99999.0, -99999.0]

        for _, element in enumerate(data):
            waypoint = [element[1], element[2]]

            if not np.allclose(waypoint, prev):
                waypoints.append(waypoint)

            prev = waypoint

        route = KDTree(waypoints)
    except IOError:
        rospy.logerr("File {} not found".format(route_file))
    except Exception as e:
        rospy.logerr("An error occurred parsing {}: {}".format(route_file, e))

    if route:
        wp = 10
        rospy.logdebug(
            "First {} waypoints of the route:\n{}".format(wp, route.data[:wp])
        )
    else:
        rospy.logwarn("No route parsed from file")

    return route


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
        ori_quat = (
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w
        )
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
        ns = rospy.get_namespace()
        rospy.loginfo_once(
            "Initializing map updater for namespace '{}'".format(ns)
        )

        # retrieve ros parameters
        self.steps = rospy.get_param("~steps")  # Num of waypoints in the track
        self.distance = rospy.get_param("~distance")  # dist btw 2 waypoints
        freq = rospy.get_param("~update_frequency")
        exit_time = rospy.get_param("~exit_time")
        self.max_speed = rospy.get_param("~max_speed")
        self.opp_speed = rospy.get_param("~opp_speed")
        self.plan_horizon = rospy.get_param("~plan_horizon")

        # state information
        self.stateReady = False
        self.state = odom_state()
        self.ado_stateReady = False
        self.ado_state = odom_state()

        self.current_stage = 1  # 1 -- start and circle     2 -- existing

        # path information
        self.ego_track_info = Path()
        self.ado_track_info = Path()
        self.global_path = get_mcity_route('ego_trajectory_event1.csv')

        # service proxy to get a path update from carla world
        rospy.wait_for_service("get_path", 8.0)
        self.get_path_handle = rospy.ServiceProxy('get_path', GetAvailablePath)

        # Subscribers for ego and opponent vehicle odometry
        rospy.Subscriber(
            "MSLcar0/ground_truth/odometry",
            Odometry,
            self.odom_cb
        )
        rospy.Subscriber(
            "MSLcar1/ground_truth/odometry",
            Odometry,
            self.ado_odom_cb
        )

        # Publishers for track information of ego and opponent
        self.ego_track_pub = rospy.Publisher(
            "MSLcar0/mpc/ego_track_information",
            Path,
            queue_size=10
        )
        self.ado_track_pub = rospy.Publisher(
            "MSLcar0/mpc/ado_track_information",
            Path,
            queue_size=10
        )

        # Class timers
        self.update_timer = rospy.Timer(
            rospy.Duration(1.0/freq),
            self.timer_cb
        )
        self.exit_timer = rospy.Timer(
            rospy.Duration(exit_time),
            self.exit_cb
        )

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
        track_width = 5.0

        _, idx = self.global_path.query([position_x, position_y])
        if idx < 2:
            track_center = self.global_path.data[idx: idx + 20, :].T.copy()
        else:
            track_center = self.global_path.data[idx - 2: idx + 18, :].T.copy()

        return track_center, track_width

    def update_ego_track(self):
        ego_pos = self.state.get_position()
        track_c, track_w = self.get_path_from_position(ego_pos[0], ego_pos[1])
        self.ego_track_info.header.stamp = rospy.Time.now()
        self.ego_track_info.header.frame_id = "map"
        self.ego_track_info.poses = []

        for i in range(self.steps):
            pose_s = PoseStamped()
            pose_s.header = self.ego_track_info.header
            pose_s.pose.position.x = track_c[0, i]
            pose_s.pose.position.y = track_c[1, i]
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
            pose_s.pose.position.x = track_c[0, i]
            pose_s.pose.position.y = track_c[1, i]
            self.ado_track_info.poses.append(pose_s)

        # append the track width info as the last element of path.poses
        track_w_pose = PoseStamped()
        track_w_pose.header = self.ado_track_info.header
        track_w_pose.pose.position.x = track_w
        self.ado_track_info.poses.append(track_w_pose)

    def exit_cb(self, event):
        # change stage from stay/enter the circle to exit the circle
        if self.current_stage == 1:
            self.current_stage = 2
        print(" SWITCHED TO EXIT MODE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

    def update_ego_track_exit(self):
        # print(" UPDATE EXIT TRACK !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        ego_pos = self.state.get_position()
        track_c, track_w = self.get_exit_path(ego_pos[0], ego_pos[1])
        self.ego_track_info.header.stamp = rospy.Time.now()
        self.ego_track_info.header.frame_id = "map"
        self.ego_track_info.poses = []

        for i in range(self.steps):
            pose_s = PoseStamped()
            pose_s.header = self.ego_track_info.header
            pose_s.pose.position.x = track_c[0, i]
            pose_s.pose.position.y = track_c[1, i]
            self.ego_track_info.poses.append(pose_s)

        # append the track width info as the last element of path.poses
        track_w_pose = PoseStamped()
        track_w_pose.header = self.ego_track_info.header
        track_w_pose.pose.position.x = track_w
        self.ego_track_info.poses.append(track_w_pose)

    def get_exit_path(self, position_x, position_y):
        track_center = np.zeros((2, self.steps))
        track_width = 5.0
        pos_the = np.arctan2(position_y, position_x)

        if pos_the > 0:    # if the car is currently in the other half circle
            the = np.linspace(pos_the-0.5, pos_the - 0.5 + np.pi, self.steps)
            radius = 21.75

            for i in range(self.steps):
                track_center[:, i] = [
                    radius*np.cos(the[i]), radius*np.sin(the[i])
                ]
        else:  # if the car is in existing half circle
            if position_x < 30:  # still use our stored path information
                _, idx = self.exit_tree.query([position_x, position_y])

                if idx < 2:
                    track_center = self.exit_tree.data[idx: idx + 20, :].T.copy()
                else:
                    track_center = self.exit_tree.data[idx - 2: idx + 18, :].T.copy()
            else:
                current_pose = Pose()
                current_pose.position.x = position_x
                current_pose.position.y = position_y

                try:
                    track_width = 3.2
                    track_center = np.zeros((2, self.steps))
                    path_list_resp = self.get_path_handle(current_pose)
                    path = path_list_resp.paths.paths[0]
                    # print(" this is our beloved path", path)

                    for i in range(self.steps - 2):
                        track_center[0,i+2] = path.poses[i].pose.position.x
                        track_center[1,i+2] = path.poses[i].pose.position.y

                    track_center[0, 1] = 2*track_center[0, 2] - track_center[0, 3]
                    track_center[1, 1] = 2*track_center[1, 2] - track_center[1, 3]
                    track_center[0, 0] = 2*track_center[0, 1] - track_center[0, 2]
                    track_center[1, 0] = 2*track_center[1, 1] - track_center[1, 2]
                except rospy.ServiceException, e:
                    print "Service call failed: %s" % e
                # print(" track center ", track_center)

        return track_center, track_width

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
