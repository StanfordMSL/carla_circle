#!/usr/bin/env python

# author: simonlc@stanford.edu

from map_updater import OdometryState

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from visualization_msgs.msg import Marker


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
        self.visualize_trajectory(msg.child_frame_id)

        return traj_msg

    def visualize_trajectory(self, label):
        '''
        Visualizes the last predicted trajectory in RVIZ.
        '''
        marker = Marker()
        marker.ns = label
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'map'
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.6
        marker.scale.y = 0.6
        marker.scale.z = 0.6
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for pose_stamped in self.predicted_traj.poses:
            marker.points.append(pose_stamped.pose.position)

        # Create a publisher for visualization
        pred_pub = rospy.Publisher(
            '/carla/vehicle/{}/trajectory'.format(label),
            Marker,
            queue_size=1
        )
        pred_pub.publish(marker)
