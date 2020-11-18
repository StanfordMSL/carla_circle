#!/usr/bin/env python

# author: mingyuw@stanford.edu

import rospy
import tf
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from carla_circle.msg import PathArray
from geometry_msgs.msg import Transform, Vector3, Quaternion, Twist

import numpy as np
import math

from map_updater import odom_state


class TrajectoryPlanner:
    MAX_ACCELERATION = 3
    MAX_ACCELERATION_ANG = 3 / 19.5

    def __init__(self):
        rospy.init_node("planner", anonymous=True)

        # retrieve ros parameters
        self.time_step = rospy.get_param("~time_step")
        self.horizon = rospy.get_param("~horizon")
        self.steps = int(self.horizon / self.time_step)
        self.reference_speed = rospy.get_param("~reference_speed")

        # class attributes
        self.predicted_traj = PathArray()  # predicted trajs for relevant cars
        # predicted _trajectories
        # each path in PathArray.paths has self.steps poses
        # the first pose start from the current location of the relevant car
        # the distance between sequential poses is speed(car) * self.time_step
        # this information is used to infer the possible time when the relevant
        # cars enter the roundabout

        self.circle_radius = 19.5  # radius of the circle trajectory
        self.circle_center = [-0.5, -0.3]  # center offset of the circle traj

        self.stateReady = False
        self.state = odom_state(self.circle_center, self.circle_radius)

        # Subscribers for ego vehicle odometry and predicted trajectory
        rospy.Subscriber(
            "/carla/ego_vehicle/odometry",
            Odometry,
            self.odom_cb
        )
        rospy.Subscriber(
            "/carla/predicted_traj",
            PathArray,
            self.predicted_traj_cb
        )

        # Publisher for desired waypoints
        self.trajectory_pub = rospy.Publisher(
            "/carla/ego_vehicle/desired_waypoints",
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

    def predicted_traj_cb(self, msg):
        self.predicted_traj = msg

    def timer_cb(self, event):
        '''
        first compute the expected time to enter the interacting area for the
        ego car
        1. then check if the time interval overlaps with the entering time for
        relevant cars
        2. if not, then plan the trajectory like there is no other car
        3. if yes, then slow down the vehicle
        '''
        if self.stateReady:
            # print("state is ready!!!!!!!!!!")
            pos_x, pos_y = self.state.get_position()
            current_speed = self.state.get_speed()
            # to avoid oscillatory behavior
            # current_speed = self.reference_speed
            current_ang = self.state.get_polar_angle()
            # rad/sec, speed moving in central angle
            ang_speed = current_speed / self.circle_radius

            # -------------------------------
            # check for possible interaction
            # --------------------------------
            def get_ang_area(x_position, y_position):
                ang_max = 0
                ang_min = 0
                if x_position > 0 and y_position > 0:
                    ang_min = np.arctan2(
                        20 - self.circle_center[1],
                        -5 - self.circle_center[0]
                    )
                    ang_max = np.arctan2(
                        15 - self.circle_center[1],
                        -15 - self.circle_center[0]
                    )
                elif x_position > 0 and y_position <= 0:
                    ang_min = np.arctan2(
                        5 - self.circle_center[1],
                        20 - self.circle_center[0]
                    )
                    ang_max = np.arctan2(
                        15 - self.circle_center[1],
                        15 - self.circle_center[0]
                    )
                elif x_position <= 0 and y_position <= 0:
                    ang_min = np.arctan2(
                        -20 - self.circle_center[1],
                        5 - self.circle_center[0]
                    )
                    ang_max = np.arctan2(
                        -15 - self.circle_center[1],
                        15 - self.circle_center[0]
                    )
                return ang_min, ang_max

            def enter_area_time(x_position, y_position, ang_speed):
                '''
                based on current position and speed, compute the t_min and
                t_max in the interested area
                '''
                t_min = 0
                t_max = 0
                relevant_flag = False
                ang_min, ang_max = get_ang_area(x_position, y_position)

                if np.linalg.norm([ang_min**2 + ang_max**2]) > 0.05:
                    t_min = (ang_min - current_ang) / ang_speed
                    t_max = (ang_max - current_ang) / ang_speed
                    relevant_flag = True

                return t_min, t_max, relevant_flag

            '''
            given current ego location and relevant paths, compute the t_min
            and t_max
            that all the path may occupy the interacting area
            if no path occupies the interacting area, then set the
            relevant_flag to False
            otherwise set flag to True
            '''
            def relevant_enter_area_time(x_position, y_position, path):
                # path is nav_msgs/Path message
                # this function is only entered if an interacting area exits
                ang_min, ang_max = get_ang_area(x_position, y_position)
                assert(ang_max > ang_min)

                def get_angle(x, y):
                    return np.arctan2(
                        y - self.circle_center[1],
                        x - self.circle_center[0]
                    )

                t_min = 0
                t_max = 0
                relevant_flag = False

                for i in range(self.steps):
                    curr_t = self.time_step * i
                    curr_pose_angle = get_angle(
                        path.poses[i].pose.position.x,
                        path.poses[i].pose.position.y
                    )

                    if curr_pose_angle > ang_min and not relevant_flag:
                        t_min = curr_t
                        relevant_flag = True

                    if curr_pose_angle < ang_max and relevant_flag:
                        t_max = curr_t

                return t_min, t_max, relevant_flag

            t_min, t_max, flag = enter_area_time(pos_x, pos_y, ang_speed)

            if flag:
                other_min_list = []
                other_max_list = []
                for path in self.predicted_traj.paths:
                    (
                        other_tmin,
                        other_tmax,
                        relevant_flag
                     ) = relevant_enter_area_time(pos_x, pos_y, path)

                    if relevant_flag:
                        other_min_list.append(other_tmin)
                        other_max_list.append(other_tmax)

                def overlap(inter1, inter2):
                    min1, max1 = inter1
                    min2, max2 = inter2
                    min = np.max([min1, min2])
                    max = np.min([max1, max2])

                    if min < max - 0.01:
                        return [min, max]
                    else:
                        return [0, 0]

                if len(other_max_list) != 0 and len(other_min_list) != 0:
                    t_max_other = np.min(
                        [np.max(other_max_list), self.horizon]
                    )
                    t_min_other = np.min(other_min_list)
                    # print("other cars will enter the area !!!!!!!!!!!!!!!!!")
                    # print("occupy time interval", t_min_other, t_max_other)
                    overlap_min, overlap_max = overlap(
                        [t_min, t_max],
                        [t_min_other, t_max_other]
                    )
                else:
                    overlap_min, overlap_max = 0, 0

                if math.sqrt(overlap_min**2 + overlap_max**2) < 0.1:
                    interaction_flag = False
                else:
                    interaction_flag = True
            else:
                interaction_flag = False

            if not interaction_flag:
                # -------------------------------
                # do not consider other cars
                # --------------------------------
                # print("planner: do not consider other cars")
                traj_msg = MultiDOFJointTrajectory()
                traj_msg.header.stamp = rospy.Time.now()
                traj_msg.header.frame_id = 'map'
                traj_msg.points = []
                angle_increment = ang_speed * self.time_step

                for i in range(self.steps):
                    ang = current_ang + angle_increment * i
                    traj_point = MultiDOFJointTrajectoryPoint()
                    traj_point.transforms = [
                        Transform(
                            Vector3(
                                self.circle_radius * np.cos(ang) +
                                self.circle_center[0],
                                self.circle_radius * np.sin(ang) +
                                self.circle_center[1], 0
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
            else:
                # ------------------------------
                # consider other cars
                # enter the area 1 sec after t_min_other
                # -------------------------------
                print(
                    "entering 1 seconds after", t_min_other,
                    "--------------------------"
                )
                print("this is the other car entering time", t_min_other)
                print("this is the ego entering time", t_min)
                if t_min < t_min_other - 0.2:
                    enter_time = t_min
                elif t_min > t_min_other - 0.2 and t_min < t_min_other + 0.3:
                    enter_time = t_min_other - 0.2
                elif t_min > t_min_other + 0.3 and t_min < t_min_other + 2.5:
                    enter_time = t_min_other + 2.5
                else:
                    enter_time = t_min

                print("this is final entering time", enter_time)
                ang_min, ang_max = get_ang_area(pos_x, pos_y)
                print(
                    "for postion", pos_x, pos_y,
                    "relevant angle is ", ang_min, ang_max
                )
                print("current ang", current_ang)
                ang_dist = ang_min - current_ang
                print("ang to travel before ", ang_dist)
                ang_dcc = (
                    2 * (ang_dist - ang_speed * enter_time) / (enter_time**2)
                )
                print("current angle velocity", ang_speed)
                print("this is desired deacceleration", ang_dcc)
                ang_dcc = np.max([ang_dcc, -self.MAX_ACCELERATION_ANG])
                decc_steps = int(enter_time / self.time_step) + 2
                print("need to decelerate at ang acceleration", ang_dcc)
                min_ang_speed = (
                    ang_speed + decc_steps * self.time_step * ang_dcc
                )
                print(
                    "this is the min speed", min_ang_speed * self.circle_radius
                )
                acc_steps = self.steps - decc_steps
                print("this is the steps left for acceleration", acc_steps)
                if acc_steps != 0:
                    ang_acc = np.min(
                        [
                            self.MAX_ACCELERATION_ANG,
                            (
                                self.reference_speed/self.circle_radius -
                                min_ang_speed
                            ) / (acc_steps * self.time_step)
                        ]
                    )
                    print("this is the acc ang:", ang_acc)

                traj_msg = MultiDOFJointTrajectory()
                traj_msg.header.stamp = rospy.Time.now()
                traj_msg.header.frame_id = 'map'
                traj_msg.points = []
                a = current_ang  # polar coordinate angle
                ang_s = ang_speed  # angular speed

                for i in range(self.steps):
                    if i <= decc_steps:
                        a = (
                            a + ang_s * self.time_step +
                            0.5 * ang_dcc * (self.time_step)**2
                        )
                        ang_s = ang_s + self.time_step * ang_dcc
                        ang_s = np.max(
                            [np.min([ang_s, 6.0/self.circle_radius]), 0]
                        )
                        s = ang_s * self.circle_radius
                        pt = MultiDOFJointTrajectoryPoint()
                        pt.transforms = [
                            Transform(
                                Vector3(
                                    self.circle_radius * np.cos(a) - 0.5,
                                    self.circle_radius * np.sin(a) - 0.3,
                                    0
                                ),
                                Quaternion(0, 0, 0, 1)
                            )
                        ]
                        pt.velocities = [
                            Twist(Vector3(s, 0, 0), Vector3(0, 0, 0))
                        ]
                        traj_msg.points.append(pt)

                        if t_min < t_min_other + 1 and t_min > t_min_other - 1:
                            print(
                                "appending ", i, "th point, velocity", s,
                                "ang vel ", ang_s, "ang", a
                            )
                    else:
                        a = (
                            a + ang_s * self.time_step +
                            0.5 * ang_acc * self.time_step**2
                        )
                        ang_s = ang_s + ang_acc * self.time_step
                        ang_s = np.max(
                            [np.min([ang_s, 6.0/self.circle_radius]), 0]
                        )
                        s = ang_s * self.circle_radius
                        pt = MultiDOFJointTrajectoryPoint()
                        pt.transforms = [
                            Transform(
                                Vector3(
                                    self.circle_radius * np.cos(a) - 0.5,
                                    self.circle_radius * np.sin(a) - 0.3,
                                    0
                                ),
                                Quaternion(0, 0, 0, 1)
                            )
                        ]
                        pt.velocities = [
                            Twist(Vector3(s, 0, 0), Vector3(0, 0, 0))
                        ]
                        traj_msg.points.append(pt)

                        if t_min < t_min_other + 1 and t_min > t_min_other - 1:
                            print(
                                "appending ", i, "th point, velocity", s,
                                "ang vel ", ang_s, "ang", a
                            )

            self.trajectory_pub.publish(traj_msg)


if __name__ == '__main__':
    try:
        planner = TrajectoryPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
