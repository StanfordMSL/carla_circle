#!/usr/bin/env python


import rospy
import tf
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory
from ackermann_msgs.msg import AckermannDriveStamped

from scipy.spatial import KDTree
import numpy as np
import timeit

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
        ori_quat = (msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w)
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

class AckermannController:
    def __init__(self):
        rospy.init_node("controller", anonymous=True)

        # retrieve ros parameters
        self.traj_horizon = rospy.get_param("~horizon")
        self.traj_steps = rospy.get_param("~plan_steps")
        self.traj_time_step = float(self.traj_horizon) / self.traj_steps
        ctrl_freq = rospy.get_param("~ctrl_freq")

        # state information
        self.stateReady = False
        self.state = odom_state()

        # path tracking information
        self.pathReady = False
        self.path = np.zeros(shape=(self.traj_steps, 2))   # absolute position
        self.path_tree = KDTree(self.path)
        self.vel_path = np.zeros(shape=(self.traj_steps, 2))

        # # PID controller parameter
        # self.pid = None
        # self.pid_motor_offset = rospy.get_param("~motor_offset")
        # self.pid_ff_prop = rospy.get_param("~ff_prop")
        # self.pid_fb_prop = rospy.get_param("~fb_prop")
        # self.pid_brake = rospy.get_param("~brake")
        self.pid_str_prop = rospy.get_param("~str_prop")

        # self.debug_counter = 0

        # subscribers, publishers
        rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, self.odom_cb)
        rospy.Subscriber("desired_waypoints", MultiDOFJointTrajectory, self.desired_waypoints_cb)
        # self.target_pub = rospy.Publisher("viz/target_pt", Marker, queue_size=10)
        self.command_pub = rospy.Publisher("/carla/ego_vehicle/ackermann_cmd", AckermannDriveStamped, queue_size=10)
        self.ctrl_timer = rospy.Timer(rospy.Duration(1.0/ctrl_freq), self.timer_cb)

    def desired_waypoints_cb(self, msg):

        for i, pt in enumerate(msg.points):
            self.path[i, 0] = pt.transforms[0].translation.x
            self.path[i, 1] = pt.transforms[0].translation.y
            self.vel_path[i, 0] = pt.velocities[0].linear.x
            self.vel_path[i, 1] = pt.velocities[0].linear.y

        self.path_tree = KDTree(self.path)
        if not self.pathReady:
            self.pathReady = True


    def odom_cb(self, msg):
        self.state.update_vehicle_state(msg)
        if not self.stateReady:
            self.stateReady = True

    def timer_cb(self, event):
        cmd_msg = AckermannDriveStamped()

        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.drive.steering_angle = 0
        cmd_msg.drive.steering_angle_velocity = 0
        cmd_msg.drive.speed = 0
        cmd_msg.drive.acceleration = 0
        cmd_msg.drive.jerk = 0

        if self.pathReady and self.stateReady:
            pos_x, pos_y = self.state.get_position()

            # pick target w/o collision avoidance, closest point on the traj and one point ahead
            _, idx = self.path_tree.query([pos_x, pos_y])
            if idx < self.traj_steps - 1:
                target_pt = self.path_tree.data[idx + 1, :]
                target_vel = self.vel_path[idx + 1, :]
            else:
                target_pt = self.path_tree.data[-1, :]
                target_vel = self.vel_path[-1, :]
                print("at the end of the desired waypoits!!!")

            target_speed = np.linalg.norm(self.target_vel)

            steer = self.compute_ackermann_cmd(target_pt)
            ackermann_cmd.drive.steering_angle = steer
            ackermann_cmd.drive.speed = target_speed
            ackermann_cmd.drive.acc = 3
        self.command_pub.publish(ackermann_cmd)

    def compute_mavros_cmd(self, target_pt, target_speed):
        pos_x, pos_y, yaw = self.state.get_position()

        egoOri = np.array([np.cos(yaw), np.sin(yaw), 0])
        rel_pos = np.array([target_pt[0] - pos_x, target_pt[1] - pos_y, 0])
        rel_pos_norm = np.linalg.norm(rel_pos)
        if target_speed == 0 or rel_pos_norm < 0.02:
            steer = 0
        else:
            rel_pos_unit = rel_pos / np.linalg.norm(rel_pos)
            rot = np.cross(rel_pos_unit, egoOri)
            steer = rot[2] * self.pid_str_prop


        return steer


if __name__ == '__main__':
    try:
        controller = AckermannController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
