#!/usr/bin/env python

# author: simonlc@stanford.edu

from map_updater import OdometryState

import rospy
import tf
from nav_msgs.msg import Odometry, Path
from carla_circle.msg import PathArray
from geometry_msgs.msg import Point, Pose, PoseStamped

import numpy as np
import math
import timeit


class TrajectoryPredictor:
    '''
    A predictor class that can be used for predicting an object's trajectory
    based on current Odometry information of the object.
    '''
    def __init__(self, time_step, horizon):
        '''
        Initializes the class.

        Parameters
        ----------
        time_step : float
            The time between each consecutive pose in the trajectory.
        horizon : float
            The time horizon in which the prediction will be calculated.
        '''
        # Base path prediction parameters
        self.time_step = time_step
        self.horizon = horizon
        self.steps = int(self.horizon / self.time_step)

        # Class attributes
        self.predicted_traj = Path()
        self.state = OdometryState()

    def update_prediction(self, msg):
        '''
        Updates the predicted trajectory of the object.

        Parameters
        ----------
        msg : Odometry
            The current Odometry information of the object in which to predict
            its trajectory.

        Output
        ---------------------
        predicted Path based on current position and velocity
        '''
        self.state.update_vehicle_state(msg)

        pos_x, pos_y = self.state.get_position()
        velocity = self.state.get_velocity()

        traj_msg = Path()
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.header.frame_id = 'map'
        traj_msg.poses = []

        for k in range(self.steps + 1):
            # Fill position and velocity at time step k in the traj_msg
            dt = k*self.time_step
            xk = pos_x + dt*velocity[0]
            yk = pos_y + dt*velocity[1]

            # How do we want to stamp our messages? ROSTime+delta? Just delta?
            # Just ROSTime (default value when creating PoseStamped object?)
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = dt
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose = Pose(
                Point(xk, yk, self.state.z),
                self.state.ori_quat,
            )
            traj_msg.poses.append(pose_stamped)

        self.predicted_traj = traj_msg
        return traj_msg
