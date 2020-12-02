#!/usr/bin/env python

# author: mingyuw@stanford.edu

import rospy
from nav_msgs.msg import Odometry, Path
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Vector3, Quaternion, Twist
from derived_object_msgs.msg import ObjectArray
import numpy as np

from map_updater import OdometryState
from predictor_module import TrajectoryPredictor

# Import some constants
from constants import MODE_NORMAL, MODE_EMERGENCY
from constants import MAX_ACCELERATION, MAX_DECELERATION


class TrivialPlanner:
    '''
    A ROS node used to publish a trajectory for ego vehicle.

    Minimum planning task: takes in a path (possibly lane center), and
    interpolates it based on current velocity to get a trajectory,
    '''
    def __init__(self):
        rospy.init_node("planner", anonymous=True)

        # retrieve ros parameters
        rolename = rospy.get_param("~rolename")
        self.time_step = rospy.get_param("~time_step")
        self.horizon = rospy.get_param("~horizon")
        self.steps = int(self.horizon / self.time_step)
        self.speed_limit = rospy.get_param("~max_speed")
        self.freq = rospy.get_param("~update_frequency")

        # class attributes
        self.track = Path()  # predicted trajs for ego cars
        self.track_ready = False

        self.state = OdometryState()
        self.stateReady = False

        # list of odometry information of other vehicles in the simulation
        self.other_agents = []

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
        rospy.Subscriber(
            "/carla/" + rolename + "/objects",
            ObjectArray,
            self.other_agents_cb
        )

        # Publisher for desired waypoints
        self.trajectory_pub = rospy.Publisher(
            "ego/command/trajectory",
            MultiDOFJointTrajectory,
            queue_size=10
        )

        # Main timer for planner
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.freq),
            self.timer_cb
        )

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

    def other_agents_cb(self, msg):
        self.other_agents = []
        objects = msg.objects
        for obj in objects:
            # obj is of type Object
            odo_msg = Odometry()

            odo_msg.header = obj.header
            odo_msg.child_frame_id = '{:03d}'.format(obj.id)
            odo_msg.pose.pose = obj.pose
            odo_msg.twist.twist = obj.twist
            self.other_agents.append(odo_msg)

    def get_track_info(self):
        num_track_points = len(self.track.poses) - 1
        track_center = np.zeros((2, num_track_points))
        track_length = np.zeros((num_track_points, ))
        track_dir = np.zeros((2, num_track_points))
        track_width = self.track.poses[num_track_points].pose.position.x

        for i in range(num_track_points):
            track_center[0, i] = self.track.poses[i].pose.position.x
            track_center[1, i] = self.track.poses[i].pose.position.y
            if i == 0:
                track_length[i] = 0
            else:
                track_length[i] = track_length[i - 1] + np.linalg.norm(
                    track_center[:, i] - track_center[:, i - 1]
                )

        for i in range(num_track_points):
            if i < num_track_points - 1:
                track_dir[:, i] = (
                    track_center[:, i + 1] - track_center[:, i]
                ) / np.linalg.norm(track_center[:, i + 1] - track_center[:, i])
            else:
                track_dir[:, i] = track_dir[:, i - 1]

        return track_center, track_width, track_dir, track_length

    def timer_cb(self, event):
        # Interpolate path using max speed or current speed
        if self.stateReady and self.track_ready:
            rospy.set_param('ctr_mode', MODE_NORMAL)
            pos_x, pos_y = self.state.get_position()
            speed_limit = self.speed_limit
            current_speed = self.state.get_speed()

            (
                track_center, track_width, track_dir, track_length
            ) = self.get_track_info()

            # Based on position of other agents, decide if we should follow the
            # current speed or stop the vehicle
            stop_flag = False

            # If we follow the current speed
            target_speed = np.array([speed_limit for i in range(self.steps)])
            interpolate_target_distance = np.array(
                [i * speed_limit * self.time_step for i in range(self.steps)]
            )
            initial_offset = np.dot(
                [pos_x, pos_y] - track_center[:, 0],
                track_dir[:, 0]
            )    # Distance ego is ahead of track start point
            track_length = track_length - initial_offset
            desired_trajectory = np.zeros((2, self.steps))
            desired_trajectory[0, :] = np.interp(
                interpolate_target_distance,
                track_length,
                track_center[0, :]
            )
            desired_trajectory[1, :] = np.interp(
                interpolate_target_distance,
                track_length,
                track_center[1, :]
            )

            # Any possible collisions along the current trajectory?
            for odo in self.other_agents:
                pred_trajectory = self.traj_predictor.update_prediction(odo)

                if self.check_interaction(pred_trajectory, desired_trajectory):
                    stop_flag = True
                    break

            if stop_flag:
                # Apply braking maneuver
                rospy.set_param('ctr_mode', MODE_EMERGENCY)
                target_speed = np.array(
                    [
                        max(
                            (
                                current_speed -
                                MAX_DECELERATION*i*self.time_step
                            ),
                            0.0
                        )
                        for i in range(self.steps)
                    ]
                )
                interpolate_target_distance = np.zeros((self.steps))

                for i in range(1, self.steps):
                    interpolate_target_distance[i] = (
                        interpolate_target_distance[i - 1] +
                        self.time_step * target_speed[i - 1]
                    )

                desired_trajectory = np.zeros((2, self.steps))
                desired_trajectory[0, :] = np.interp(
                    interpolate_target_distance,
                    track_length,
                    track_center[0, :]
                )
                desired_trajectory[1, :] = np.interp(
                    interpolate_target_distance,
                    track_length,
                    track_center[1, :]
                )

            traj_msg = MultiDOFJointTrajectory()
            traj_msg.header.stamp = rospy.Time.now()
            traj_msg.header.frame_id = 'map'
            traj_msg.points = []
            rospy.logdebug("Target speed: {}".format(target_speed))

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
                        Vector3(target_speed[i], 0.0, 0.0),
                        Vector3(0.0, 0.0, 0.0)
                    )
                ]
                traj_msg.points.append(traj_point)

            self.trajectory_pub.publish(traj_msg)

    def check_interaction(self, other_traj, ego_traj):
        '''
        Checks the trajectories of two objects and determines if there is a
        potential collision.

        Assumes dt is our planner's time_step.

        Parameters
        ----------
        other_traj: Path
            The predicted trajectory of one other vehicle, starting from its
            current position.
        ego_traj: 2 x n numpy array
            The trajectory of ego vehicle, starting from its current position.
        '''
        for t in range(self.steps - 1):
            other_position = [
                other_traj.poses[t].pose.position.x,
                other_traj.poses[t].pose.position.y
            ]
            positive_direction = ego_traj[:, t + 1] - ego_traj[:, t]

            if (
                np.linalg.norm(other_position - ego_traj[:, t]) < 3.25
                and np.dot(
                    other_position - ego_traj[:, t],
                    positive_direction
                ) > 0
            ):
                rospy.logwarn("Potential collision...")
                return True

        return False


if __name__ == '__main__':
    try:
        planner = TrivialPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
