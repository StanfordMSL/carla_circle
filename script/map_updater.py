#!/usr/bin/env python

# map_updater.py
# author: mingyuw@stanford.edu

import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path, Odometry


import numpy as np
from numpy import genfromtxt
from scipy.spatial import KDTree

import os
import random

import carla

def get_mcity_route(filename):
    '''
    Returns a collection of waypoints representing the global plan/route in
    which the ego vehicle should follow.

    The parsing of the waypoints is done in such a way to try and remove
    consecutive duplicates from the csv file. Since these files seem to be the
    obtained from a recording of Michigan's vehicle, the csv has several
    waypoints consecutively from where the vehicle is not moving.

    Note: csv file data is given in lefthanded coordinates, while ROS and all our
    code use righthanded coordinates. Thus, we flip the sign of y coordintes while
    reading in waypoints.

    Parameters
    ----------
    filename : str
        The name of the file that will be parsed to obtain the route.

    Returns
    -------
    KDTree
        A KDTree object that contains the collection of waypoints representing
        the global plan for the ego vehicle.
    '''
    resource_folder = os.path.join(os.path.dirname(__file__), "../resource/")
    route_file = os.path.join(resource_folder, filename)
    route = None

    try:
        rospy.loginfo("Reading from file {}".format(route_file))
        data = genfromtxt(route_file, delimiter=',', skip_header=0)
        waypoints = []
        prev = [-99999.0, -99999.0]

        for _, element in enumerate(data):
            waypoint = [element[1], -element[2]]

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
    '''
    This class stores an instantaneous odometry state of the vehicle. All units
    are SI.
    '''
    def __init__(self):
        '''
        Initializes the class variables.
        '''
        self.time = None
        self.x = None
        self.y = None
        self.yaw = None
        self.vx = None
        self.vy = None
        self.speed = None

    def update_vehicle_state(self, odom_msg):
        '''
        Updates the member variables to the instantaneous odometry state of a
        vehicle.

        Parameters
        ----------
        odom_msg : Odometry
            A message containing the current odometry state of the vehicle.
        '''
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

    def get_position(self):
        '''
        Returns the stored global position :math:`(x, y)` of the vehicle
        center.

        Returns
        -------
        Array
            The 2D position (global) of the vehicle.
        '''
        return [self.x, self.y]

    def get_pose(self):
        '''
        Returns the stored global position :math:`(x, y)` of the vehicle center
        and heading/yaw :math:`(\theta)` with respect to the x-axis.

        Returns
        -------
        Array
            A SE(2) vector representing the global position and heading of the
            vehicle.
        '''
        return [self.x, self.y, self.yaw]

    def get_velocity(self):
        '''
        Returns the stored global velocity :math:`(v_x, v_y)` of the vehicle.

        Returns
        -------
        Array
            The 2D velocity (global) of the vehicle.
        '''
        return [self.vx, self.vy]

    def get_speed(self):
        '''
        Returns the stored global speed of the vehicle.

        Returns
        -------
        float
            The speed (global) of the vehicle.
        '''
        return self.speed


class MapUpdater:
    '''
    A ROS node used to publish the near-future route of ego vehicle.

    The node publishes at a provided frequency.
    '''
    def __init__(self):
        '''
        Initializes the class, including creating the publishers and
        subscribers used throughout the class.
        '''
        rospy.init_node("map_updater", anonymous=True)
        ns = rospy.get_namespace()
        rospy.loginfo_once(
            "Initializing map updater for namespace '{}'".format(ns)
        )

        # retrieve ros parameters
        self.steps = rospy.get_param("~steps")  # Num of waypoints in the track
        freq = rospy.get_param("~update_frequency")
        role_name = rospy.get_param("~rolename")

        # state information
        self.stateReady = False
        self.state = odom_state()
        self.ado_stateReady = False
        self.ado_state = odom_state()

        # path information
        self.ego_track_info = Path()
        self.ado_track_info = Path()
        self.global_path = get_mcity_route('ego_trajectory_event1.csv')


        # Subscribers for ego and opponent vehicle odometry
        rospy.Subscriber(
            "/carla/" + role_name + "/odometry",
            Odometry,
            self.odom_cb
        )
        rospy.Subscriber(
            "ado/ground_truth/odometry",
            Odometry,
            self.ado_odom_cb
        )

        # Publishers for track information of ego and opponent, as well as
        # global route
        self.ego_track_pub = rospy.Publisher(
            "ego_track_information",
            Path,
            queue_size=10
        )
        self.ado_track_pub = rospy.Publisher(
            "ado_track_information",
            Path,
            queue_size=10
        )
        self.global_plan_pub = rospy.Publisher(
            "{}global_plan".format(ns),
            Path,
            queue_size=1
        )

        # Guarantees the publisher is ready before attempting to publish
        while self.global_plan_pub.get_num_connections() < 1:
            pass

        self.publish_global_path()

        # Class timers
        self.update_timer = rospy.Timer(
            rospy.Duration(1.0/freq),
            self.timer_cb
        )

    def publish_global_path(self):
        '''
        Updates and publishes the global plan/path of the ego vehicle.
        '''
        global_path = Path()
        global_path.header.stamp = rospy.Time.now()
        global_path.header.frame_id = "map"
        global_path.poses = []

        for _, data in enumerate(self.global_path.data):
            pose_s = PoseStamped()
            pose_s.header = global_path.header
            pose_s.pose.position.x = data[0]
            pose_s.pose.position.y = data[1]
            global_path.poses.append(pose_s)

        self.global_plan_pub.publish(global_path)

    def odom_cb(self, msg):
        '''
        Callback function to update the odometry state of the ego vehicle
        according to the received message.

        Parameters
        ----------
        msg : Odometry
            The message containing the vehicle's odometry state.
        '''
        self.state.update_vehicle_state(msg)

        if not self.stateReady:
            self.stateReady = True

    def ado_odom_cb(self, msg):
        '''
        Callback function to update the odometry state of the opponent vehicle
        according to the received message.

        Parameters
        ----------
        msg : Odometry
            The message containing the vehicle's odometry state.
        '''
        self.ado_state.update_vehicle_state(msg)

        if not self.ado_stateReady:
            self.ado_stateReady = True

    def get_path_from_position(self, position_x, position_y):
        '''
        Returns the track width and center based on the position of the
        vehicle.

        Parameters
        ----------
        position_x : float
            The x global position of the vehicle wrt the origin of the map.
        position_y : float
            The y global position of the vehicle wrt the origin of the map.

        Returns
        -------
        (numpy.ndarray, float)
            The center of the track and the width of the track.
        '''
        track_center = np.zeros((2, self.steps))
        track_width = 3.0

        _, idx = self.global_path.query([position_x, position_y])
        if idx < 1:
            track_center = self.global_path.data[idx: idx + self.steps, :].T.copy()
        else:
            track_center = self.global_path.data[idx - 1: idx + self.steps - 1, :].T.copy()
        track_center[1,:] = track_center[1,:]

        rospy.logdebug(
            "Track:\nCenters - {}\nWidth - {}".format(
                track_center, track_width
            )
        )

        return track_center, track_width

    def update_ego_track(self):
        '''
        Updates the ego vehicle's path.

        The path is a nav_msg.Path object.
        '''
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
        '''
        Updates the opponent vehicle's path.

        The path is a nav_msg.Path object.
        '''
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

    def timer_cb(self, event):
        '''
        This is a callback for the class timer, which will be called every
        tick.

        The callback updates of track information for the ego and opponent
        vehicles.

        Parameters
        ----------
        event : rospy.TimerEvent
            The timer's tick event.
        '''
        if self.stateReady:
            self.update_ego_track()
            self.ego_track_pub.publish(self.ego_track_info)

        if self.ado_stateReady:
            self.update_ado_track()
            self.ado_track_pub.publish(self.ado_track_info)

        self.publish_global_path()

if __name__ == '__main__':
    try:
        supervisor = MapUpdater()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
