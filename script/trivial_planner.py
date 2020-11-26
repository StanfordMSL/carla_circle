#!/usr/bin/env python

# author: mingyuw@stanford.edu

import rospy
import tf
from nav_msgs.msg import Odometry, Path
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Vector3, Quaternion, Twist

import numpy as np
import math

from map_updater import OdometryState
from predictor_module import TrajectoryPredictor


class TrivialPlanner:
    '''
    A ROS node used to publish a trajectory for ego vehicle.

    Minimum planning task: takes in a path (possibly lane center), and
    interpolates it based on current velocity to get a trajectory,
    '''
    MAX_ACCELERATION = 3
    MAX_ACCELERATION_ANG = 3 / 19.5

    def __init__(self):
        rospy.init_node("planner", anonymous=True)

        # retrieve ros parameters
        rolename = rospy.get_param("~rolename")
        self.time_step = rospy.get_param("~time_step")
        self.horizon = rospy.get_param("~horizon")
        self.steps = int(self.horizon / self.time_step)
        self.reference_speed = rospy.get_param("~max_speed")

        # class attributes
        self.track = Path()  # predicted trajs for relevant cars
        self.track_ready = False

        self.stateReady = False
        self.state = OdometryState()

        # Predictor
        self.traj_predictor = TrajectoryPredictor(self.time_step, self.horizon)

        # Subscribers for ego vehicle odometry and predicted trajectory
        rospy.Subscriber(
            "/carla/" + rolename + "/odometry",
            Odometry,
            self.odom_cb
        )
        rospy.Subscriber(
            "ego_track_information",
            Path,
            self.track_cb
        )

        # Publisher for desired waypoints
        self.trajectory_pub = rospy.Publisher(
            "ego/command/trajectory",
            MultiDOFJointTrajectory,
            queue_size=10
        )

        # Main timer for planner
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)

    '''
    this function updates the ego vehicle state in class
    '''
    def odom_cb(self, msg):
        self.state.update_vehicle_state(msg)
        if not self.stateReady:
            self.stateReady = True

    def track_cb(self, msg):
        self.track = msg
        if not self.track_ready:
            self.track_ready = True

    def get_track_info(self):
        num_track_points = len(self.track.poses) - 1
        track_center = np.zeros((2, num_track_points))
        track_length = np.zeros((num_track_points,))
        track_dir = np.zeros((2,num_track_points))
        track_width = self.track.poses[num_track_points].pose.position.x
        for i in range(num_track_points):
            track_center[0, i] = self.track.poses[i].pose.position.x
            track_center[1, i] = self.track.poses[i].pose.position.y
            if i == 0:
                track_length[i] = 0
            else:
                track_length[i] = track_length[i-1] + np.linalg.norm(track_center[:,i] - track_center[:,i-1])
        for i in range(num_track_points):
            if i < num_track_points - 1:
                track_dir[:,i] = (track_center[:,i+1] - track_center[:,i])/np.linalg.norm(track_center[:,i+1] - track_center[:,i])
            else:
                track_dir[:,i] = track_dir[:,i-1]

        return track_center, track_width, track_dir, track_length

    def timer_cb(self, event):
        # interpolate path using max speed or current speed
        if self.stateReady and self.track_ready:
            pos_x, pos_y = self.state.get_position()
            speed = self.reference_speed

            track_center, track_width, track_dir, track_length = self.get_track_info()
            # use interpolation, always assume ego vehicle strictly inside the
            interpolate_target_distance = np.array([i * self.reference_speed * self.time_step  for i in range(self.steps)])
            initial_offset = np.dot([pos_x, pos_y] - track_center[:,0], track_dir[:,0])    # distance ego ahead of track start point
            track_length = track_length - initial_offset

            desired_trajectory = np.zeros((2, self.steps))
            desired_trajectory[0,:] = np.interp(interpolate_target_distance, track_length, track_center[0,:])
            desired_trajectory[1,:] = np.interp(interpolate_target_distance, track_length, track_center[1,:])
            traj_msg = MultiDOFJointTrajectory()
            traj_msg.header.stamp = rospy.Time.now()
            traj_msg.header.frame_id = 'map'
            traj_msg.points = []

            for i in range(self.steps):
                traj_point = MultiDOFJointTrajectoryPoint()
                traj_point.transforms = [
                    Transform(
                        Vector3(
                            desired_trajectory[0, i],
                            desired_trajectory[1, i],
                            0.0
                        ),
                        Quaternion(0.0, 0.0, 0.0, 1.0)
                    )
                ]
                traj_point.velocities = [
                    Twist(
                        Vector3(speed, 0.0, 0.0),
                        Vector3(0.0, 0.0, 0.0)
                    )
                ]
                traj_msg.points.append(traj_point)

            self.trajectory_pub.publish(traj_msg)


if __name__ == '__main__':
    try:
        planner = TrivialPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
