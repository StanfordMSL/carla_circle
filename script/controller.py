#!/usr/bin/env python
# author: mingyuw@stanford.edu

import rospy
import tf
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory
from ackermann_msgs.msg import AckermannDrive
from visualization_msgs.msg import Marker

from carla_msgs.msg import CarlaEgoVehicleControl
from scipy.spatial import KDTree
import numpy as np
import timeit

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
        # print(
        #     "this is the current speed @ time",
        #     self.speed,
        #     self.time
        # )


    def get_position(self):
        '''
        Returns the stored global position :math:`(x, y)` of the vehicle center.
        
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
        Array
            The speed (global) of the vehicle.
        '''
        return self.speed

class AckermannController:
    '''
    A ROS node used to control the vehicle based on some trajectory.
    Specifically, this node publishes AckermannDrive messages for the ego
    vehicle determined by a set of desired waypoints.
    '''
    def __init__(self):
        '''
        Initializes the class, including creating the used publishers and
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
        self.steer_prev = 0.0

        # state information
        self.stateReady = False
        self.state = odom_state()

        # path tracking information
        self.pathReady = False
        self.path = np.zeros(shape=(self.traj_steps, 2)) # Absolute position
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
            "MSLcar0/command/trajectory",
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
        This is a callback for the class timer, which will be called every tick.
        The callback calculates the target values of the AckermannDrive message
        and publishes the command to the ego vehicle's Ackermann control topic.
        
        Parameters
        ----------
        event : rospy.TimerEvent
            The timer's tick event.
        '''
        cmd_msg = AckermannDrive()
        current_time = rospy.get_time()
        # delta_t = current_time - self.time
        delta_t = self.time_step
        # delta_t = 0.05
        self.time = current_time
        jerk = 2.0

        # cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.steering_angle = 0.0
        cmd_msg.steering_angle_velocity = 0.0
        cmd_msg.speed = 0.0
        cmd_msg.acceleration = 0.0
        cmd_msg.jerk = jerk

        if self.pathReady and self.stateReady:
            pos_x, pos_y = self.state.get_position()

            # pick target w/o collision avoidance, closest point on the traj and one point ahead
            _, idx = self.path_tree.query([pos_x, pos_y])
            # Target point for steering
            if idx < self.traj_steps - 3:
                target_pt = self.path_tree.data[idx + 3, :]
                # str_idx = idx + 2
            else:
                target_pt = self.path_tree.data[-1, :]
                # str_idx = self.vel_path.shape[0] - 1
                print("CONTROLLER: at the end of the desired waypoits!!!")

            # Target point for velocity
            if idx < self.traj_steps:
                target_vel = self.vel_path[idx, :]
                # str_idx = idx + 2
            else:
                target_vel = self.vel_path[-1, :]
                # str_idx = self.vel_path.shape[0] - 1

            target_speed = np.linalg.norm(target_vel)

            speed_diff = target_speed - self.state.get_speed()
            # pos_diff = target_pt - self.state.get_position()
            # acceleration = cmd_msg.acceleration + 2.0*(
            #     pos_diff/self.time_step**2.0 - speed_diff/self.time_step
            # )
            acceleration = abs(speed_diff) / (2.0 * delta_t)
            cmd_msg.acceleration = np.min([1.5, acceleration])
            steer = self.compute_ackermann_steer(target_pt)
            
            steer_diff = abs(steer - self.steer_prev)

            if steer_diff >= 0.3:
                print("      0.3")
                steer = self.steer_prev
            elif steer_diff >= 0.05:
                print("          0.05")
                acceleration = 0.0
                target_speed = target_speed / 5.0

            self.steer_prev = steer

            if self.state.get_speed() - target_speed > 0.0:
                # cmd_msg.acceleration = acceleration - 5
                acceleration = -100.0
                cmd_msg.speed = target_speed / 5.0
                jerk = 1000.0
            elif target_speed - self.state.get_speed() > 0.2:
                acceleration = np.min([3.0, acceleration])
            else:
                acceleration = 0.0

            cmd_msg.steering_angle = steer
            cmd_msg.speed = target_speed
            cmd_msg.acceleration
            cmd_msg.jerk = jerk

            # Print out some debugging information
            if abs(target_speed - self.max_speed) > 2.0:
                # print("Posit diff:", pos_diff)
                print("Speed diff:", speed_diff)
                print("Current speed:", self.state.get_speed())
                print("Desired speed:", target_speed)
                print("Desired accel:", acceleration)
                print("Desired jerk:", jerk)
                print("Speed (sent):", cmd_msg.speed)
                print("Accel (sent):", cmd_msg.acceleration)
                print("Jerk  (sent):", cmd_msg.jerk)
                print("delta t:", delta_t)

            # for visualization purposes and debuging control node
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


        # self.vehicle_cmd_pub.publish(vehicle_cmd_msg)
        self.command_pub.publish(cmd_msg)


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
            print("target point too close!!!!!!!!")
            if self.steer_cache:
                return self.steer_cache
            else:
                return 0.0

        egoOri = np.array([np.cos(yaw), np.sin(yaw), 0])
        rel_pos = np.array([target_pt[0] - pos_x, target_pt[1] - pos_y, 0])
        rel_pos_norm = np.linalg.norm(rel_pos)

        rel_pos_unit = rel_pos / np.linalg.norm(rel_pos)
        rot = np.cross(rel_pos_unit, egoOri)
        steer = -rot[2] * self.pid_str_prop

        if self.steer_cache is not None:
            d_steer = (steer - self.steer_cache)
        else:
            d_steer = 0.0

        # print("steer, d_steer:", steer, d_steer)

        steer = steer + d_steer*self.pid_str_deriv
        self.steer_cache = steer

        return steer


if __name__ == '__main__':
    try:
        controller = AckermannController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
