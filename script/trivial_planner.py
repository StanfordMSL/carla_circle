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
        ori_quat = (odom_msg.pose.pose.orientation.x,
                    odom_msg.pose.pose.orientation.y,
                    odom_msg.pose.pose.orientation.z,
                    odom_msg.pose.pose.orientation.w)
        ori_euler = tf.transformations.euler_from_quaternion(ori_quat)
        self.yaw = ori_euler[2]
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


class TrivialPlanner:
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
        # predicted _trajectories
        # each path in PathArray.paths has self.steps poses
        # the first pose start from the current location of the relevant car
        # the distance between sequential poses is speed(car) * self.time_step
        # this information is used to infer the possible time when the relevant
        # cars enter the roundabout

        # self.circle_radius = 19.5  # radius of the circle trajectory
        # self.circle_center = [-0.5, -0.3]  # center offset of the circle traj

        self.stateReady = False
        self.state = odom_state(0, 0)

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
                            desired_trajectory[0,i],
                            desired_trajectory[1,i], 0
                        ),
                        Quaternion(0, 0, 0, 1)
                    )
                ]
                traj_point.velocities = [
                    Twist(
                        Vector3(self.reference_speed, 0, 0),
                        Vector3(0, 0, 0)
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
