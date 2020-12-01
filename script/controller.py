#!/usr/bin/env python
# author: mingyuw@stanford.edu

import rospy
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory
from ackermann_msgs.msg import AckermannDrive
from visualization_msgs.msg import Marker

from carla_msgs.msg import CarlaEgoVehicleInfo
from scipy.spatial import KDTree
import numpy as np

from map_updater import OdometryState

# Import some constants
from constants import MODE_EMERGENCY
from constants import MAX_ACCELERATION, MAX_DECELERATION, MAX_JERK
from constants import FREEFLOW_SPEED


class AckermannController:
    '''
    A ROS node used to control the vehicle based on some trajectory.
    Specifically, this node publishes AckermannDrive messages for the ego
    vehicle determined by a set of desired waypoints.
    '''
    def __init__(self):
        '''
        Initializes the class, including creating the publishers and
        subscribers used throughout the class.
        '''
        rospy.init_node("controller", anonymous=True)
        self.time = rospy.get_time()

        # retrieve ros parameters
        self.max_speed = rospy.get_param("~max_speed")
        self.time_step = rospy.get_param("~time_step")
        self.traj_steps = rospy.get_param("~plan_steps")
        ctrl_freq = rospy.get_param("~ctrl_freq")
        rolename = rospy.get_param("~rolename")
        rospy.loginfo_once(
            "Initializing Ackermann Controller for '{}'".format(rolename)
        )
        self.steer_prev = 0.0

        # state information
        self.stateReady = False
        self.state = OdometryState()

        # path tracking information
        self.pathReady = False
        self.path = np.zeros(shape=(self.traj_steps, 2))  # Absolute position
        self.path_tree = KDTree(self.path)
        self.vel_path = np.zeros(shape=(self.traj_steps, 2))

        self.steer_cache = None

        # PID controller parameter
        self.pid_str_prop = rospy.get_param("~str_prop")
        self.pid_str_deriv = rospy.get_param("~str_deriv")

        # Subscribers
        rospy.Subscriber(
            "/carla/" + rolename + "/odometry",
            Odometry,
            self.odom_cb
        )
        rospy.Subscriber(
            "ego/command/trajectory",
            MultiDOFJointTrajectory,
            self.desired_waypoints_cb
        )

        # Publishers
        self.command_pub = rospy.Publisher(
            "/carla/" + rolename + "/ackermann_cmd",
            AckermannDrive,
            queue_size=10
        )
        self.tracking_pt_viz_pub = rospy.Publisher(
            "tracking_point_mkr",
            Marker,
            queue_size=10
        )

        topic = "/carla/{}/vehicle_info".format(rolename)
        rospy.loginfo_once(
            "Vehicle information for {}: {}".format(
                rolename,
                rospy.wait_for_message(topic, CarlaEgoVehicleInfo)
            )
        )

        # Class timer
        self.ctrl_timer = rospy.Timer(
            rospy.Duration(1.0/ctrl_freq),
            self.timer_cb
        )

    def desired_waypoints_cb(self, msg):
        '''
        A callback function that updates the desired the desired trajectory the
        ego vehicle should follow.

        Parameters
        ----------
        msg : MultiDOFJointTrajectory
            The previously calculated trajectory from the planner.
        '''
        for i, pt in enumerate(msg.points):
            self.path[i, 0] = pt.transforms[0].translation.x
            self.path[i, 1] = pt.transforms[0].translation.y
            self.vel_path[i, 0] = pt.velocities[0].linear.x
            self.vel_path[i, 1] = pt.velocities[0].linear.y

        self.path_tree = KDTree(self.path)

        if not self.pathReady:
            self.pathReady = True

    def odom_cb(self, msg):
        '''
        Callback function to update the odometry state of the vehicle according
        to the received message.

        Parameters
        ----------
        msg : Odometry
            The message containing the vehicle's odometry state.
        '''
        self.state.update_vehicle_state(msg)

        if not self.stateReady:
            self.stateReady = True

    def timer_cb(self, event):
        '''
        This is a callback for the class timer, which will be called every
        tick.

        The callback calculates the target values of the AckermannDrive
        message and publishes the command to the ego vehicle's Ackermann
        control topic.

        Parameters
        ----------
        event : rospy.TimerEvent
            The timer's tick event.
        '''
        current_time = rospy.get_time()
        # delta_t = current_time - self.time
        # delta_t = self.time_step
        # delta_t = 0.05
        self.time = current_time
        desired_steer = 0.0
        desired_speed = 0.0
        desired_acceleration = 0.0
        desired_jerk = 0.0

        ctr_mode = rospy.get_param("ctr_mode")

        # Initializes the AckermannDrive message; All values 0 unless specified
        cmd_msg = AckermannDrive()

        if self.pathReady and self.stateReady and ctr_mode != MODE_EMERGENCY:
            pos_x, pos_y = self.state.get_position()

            # Pick target w/o collision avoidance. Find the closest point in
            # the trajectory tree.
            _, idx = self.path_tree.query([pos_x, pos_y])

            # Steering target. Some points ahead of closest point.
            points_ahead = 4
            if idx < self.traj_steps - points_ahead:
                target_pt = self.path_tree.data[idx + points_ahead, :]
            else:
                target_pt = self.path_tree.data[-1, :]
                rospy.logwarn("At the end of the desired waypoints!")

            # Velocity target. Use the desired velocity from closest point.
            points_ahead = 0
            if idx < self.traj_steps - points_ahead:
                target_vel = self.vel_path[idx + points_ahead, :]
            else:
                target_vel = self.vel_path[-1, :]

            desired_speed = np.linalg.norm(target_vel)
            current_speed = self.state.get_speed()
            speed_diff = desired_speed - current_speed
            rospy.logdebug("Speed difference: {}".format(speed_diff))

            desired_steer = self.compute_ackermann_steer(target_pt)
            steer_diff = abs(desired_steer - self.steer_prev)
            rospy.logdebug("Steering difference: {}".format(steer_diff))

            if steer_diff >= 0.3:
                rospy.logdebug("Large difference; Use last steering command")
                desired_steer = self.steer_prev

            self.steer_prev = desired_steer

            cmd_msg.steering_angle = desired_steer
            cmd_msg.speed = desired_speed
            cmd_msg.acceleration = desired_acceleration
            cmd_msg.jerk = desired_jerk

            # For visualization purposes and debuging control node
            self.publish_markers(target_pt)

        # self.vehicle_cmd_pub.publish(vehicle_cmd_msg)
        self.command_pub.publish(cmd_msg)

    def compute_ackermann_long_params(self, target_velocity):
        '''
        Calculates the desired longitudinal values for the vehicle to smoothly
        get to the desired speed.

        Parameters
        ----------
        target_velocity : float
            The target velocity (global) in which the vehicle should drive.

        Returns
        -------
        (float, float, float)
            The calculated desired speed, acceleration and jerk of the vehicle,
            respectively.
        '''
        desired_speed = 0.0
        desired_acceleration = 0.0
        desired_jerk = 0.0

        # Calculate desired speed
        desired_speed = np.linalg.norm(target_velocity)

        # Calculate the desired acceleration
        current_speed = self.state.get_speed()
        speed_diff = desired_speed - current_speed
        acceleration = speed_diff * FREEFLOW_SPEED
        desired_acceleration = np.clip(
            acceleration, -MAX_DECELERATION, MAX_ACCELERATION
        )

        # Calculate desired jerk
        jerk = 0.0
        desired_jerk = np.clip(jerk, 0.0, MAX_JERK)

        rospy.logdebug(
            "Desired long parameters (speed, accel, jerk): {}, {}, {}".format(
                desired_speed, desired_acceleration, desired_jerk
            )
        )

        return (desired_speed, desired_acceleration, desired_jerk)

    def compute_ackermann_steer(self, target_pt):
        '''
        Calculates the desired steering command [-1, 1] for the vehicle to
        manuever towards the provided global position :math:`(x, y)`.

        Parameters
        ----------
        target_pt : Array
            The 2D target position (global) in which the vehicle should head.

        Returns
        -------
        float
            The steering command for the vehicle in the range :math:`[-1, 1]`,
            representing the max left and right control, respectively.
        '''
        pos_x, pos_y, yaw = self.state.get_pose()

        if np.linalg.norm([target_pt[0] - pos_x, target_pt[1] - pos_y]) < 1:
            rospy.logwarn("Steering target point too close!!!!!!!!")
            if self.steer_cache:
                return self.steer_cache
            else:
                return 0.0

        egoOri = np.array([np.cos(yaw), np.sin(yaw), 0])
        rel_pos = np.array([target_pt[0] - pos_x, target_pt[1] - pos_y, 0])
        rel_pos_norm = np.linalg.norm(rel_pos)

        rel_pos_unit = rel_pos / rel_pos_norm
        rot = np.cross(rel_pos_unit, egoOri)
        steer = -rot[2] * self.pid_str_prop

        if self.steer_cache is not None:
            d_steer = (steer - self.steer_cache)
        else:
            d_steer = 0.0

        steer = steer + d_steer*self.pid_str_deriv
        self.steer_cache = steer

        return steer

    def publish_markers(self, target_pt):
        mk_msg = Marker()
        mk_msg.header.stamp = rospy.Time.now()
        mk_msg.header.frame_id = 'map'
        mk_msg.pose.position.x = target_pt[0]
        mk_msg.pose.position.y = target_pt[1]
        mk_msg.type = Marker.CUBE
        mk_msg.scale.x = 3
        mk_msg.scale.y = 3
        mk_msg.scale.z = 3
        mk_msg.color.a = 1.0
        mk_msg.color.r = 1
        mk_msg.color.b = 1
        self.tracking_pt_viz_pub.publish(mk_msg)


if __name__ == '__main__':
    try:
        controller = AckermannController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
