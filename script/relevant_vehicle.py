#!/usr/bin/env python
# author: mingyuw@stanford.edu

"""
this node publishes the following information
1. relevant vehiles as derived_object_msgs ObjectArray (relevant as have possible interaction)
2. marker of relevant vehicles for visualization
3. future path of the relevant vehicles based on road geometry
4. future trajectories of the relevant vehicles based on road geometry and
    assumes that the vehicle will maintain a constant speed
"""
import rospy
import tf
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from derived_object_msgs.msg import ObjectArray, Object
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from carla_circle.msg import PathArray, Agent, AgentArray
from carla_circle.srv import GetAvailablePath

import carla

import numpy as np
import timeit
import math


class RelevantCarFilter(object):
    def __init__(self):
        rospy.init_node("relevantCarFilter", anonymous=True)

        # -----------------
        # retrieve ROS parameters
        # -------------------
        self.time_step = rospy.get_param("~time_step")
        self.horizon = rospy.get_param("~horizon")
        self.steps = int(self.horizon / self.time_step)

        # -----------------
        # setup attributes for class
        # -------------------
        rospy.wait_for_service("get_path")
        self.get_path_handle = rospy.ServiceProxy('get_path', GetAvailablePath)
        self.ego_location = carla.Location()
        self.nearby_obj = ObjectArray()
        self.ego_ready = False
        self.nearby_ready = False

        self.enable_lane_filter = True     # if true, then only vehicles entering in the inner lane
                                           # will be considered

        # -----------------
        # subscriber, publisher and timer
        # -------------------
        rospy.Subscriber("/carla/nearby_obj", ObjectArray, self.nearby_car_cb)
        rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, self.ego_vehicle_cb)
        # publisher for relevant cars
        self.relevant_pub = rospy.Publisher("/carla/relevant_obj", ObjectArray, queue_size=10)
        self.marker_pub = rospy.Publisher("/carla/viz/relevant_obj", MarkerArray, queue_size=10)
        # publishers for relevant path (not considering speed information)
        self.path_pub = rospy.Publisher("carla/predict_path", PathArray, queue_size=10)
        self.path_viz_pub = rospy.Publisher("carla/viz/predict_path", MarkerArray, queue_size=10)
        # publishers for relevant car as Agents message
        self.predict_trajectory_pub = rospy.Publisher("/carla/predicted_traj", PathArray, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_cb)

    def ego_vehicle_cb(self, msg):
        self.ego_location.x = msg.pose.pose.position.x
        self.ego_location.y = msg.pose.pose.position.y
        if not self.ego_ready:
            self.ego_ready = True

    def nearby_car_cb(self, msg):
        self.nearby_obj = msg
        if not self.nearby_ready:
            self.nearby_ready = True

    def timer_cb(self, event):
        '''
        if object is relevant given position of ego car at (pos_x, pos_y)
        returns True, else returns False
        '''
        def relevant_filter(pos_x, pos_y, object):
            # return True
            pos_car = [object.pose.position.x, object.pose.position.y]
            if not self.enable_lane_filter:
                '''
                does not filter relevant vehicles based on their lane
                '''
                if pos_x > 0 and pos_y < 0:
                    if (pos_car[0] > 15 and pos_car[1] > 0):
                    # if (pos_car[0] > 0 and pos_car[1] < -20) or (pos_car[0] > 20 and pos_car[1] > 0):
                        return True
                    return False
                elif pos_x > 0 and pos_y >= 0:
                    if (pos_car[0] < 0 and pos_car[1] > 15):
                        return True
                    return False
                elif pos_x <= 0 and pos_y >= 0:
                    if (pos_car[0] < -20 and pos_car[1]< 0):
                        return True
                    return False
                else:
                    if pos_car[0] > 0 and pos_car[1] < -20:
                        return True
                    return False
            else:
                '''
                filter vehicles based on their lane number
                currently, using road geometry, should be changed to based on closest waypoint information
                '''
                if pos_x > 0 and pos_y < 0:
                    if (pos_car[0] > 15 and pos_car[1] > 0 and pos_car[1] < 7):
                    # if (pos_car[0] > 0 and pos_car[1] < -20) or (pos_car[0] > 20 and pos_car[1] > 0):
                        return True
                    return False
                elif pos_x > 0 and pos_y >= 0:
                    if (pos_car[0] < 0 and pos_car[0] > -6 and pos_car[1] > 15)\
                            or (pos_car[0] < -5 and pos_car[0] > -10 and pos_car[1] > 20 and pos_car[1] < 25):
                        return True
                    return False
                elif pos_x <= 0 and pos_y >= 0:
                    # if (pos_car[0] < -20 and pos_car[1]< 0):
                    #     return True
                    return False
                else:
                    if (pos_car[0] > 0 and pos_car[0] < 5 and pos_car[1] < -20) \
                            or (pos_car[0] > 5 and pos_car[0] < 10 and pos_car[1] < -20 and pos_car[1] > -25):
                        return True
                    return False

        if self.ego_ready and self.nearby_ready:
            # ----------------------
            # publish derived object message for all relevant cars
            # ----------------------
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'map'
            relevant_msg = ObjectArray()
            relevant_msg.header = header
            relevant_msg.objects = [obj for obj in self.nearby_obj.objects \
                            if relevant_filter(self.ego_location.x, self.ego_location.y, obj)]
            self.relevant_pub.publish(relevant_msg)

            # ----------------------
            # predict future path for relevant cars
            # ----------------------
            future_path = PathArray()
            future_path.header.stamp = rospy.Time.now()
            future_path.header.frame_id = 'map'
            future_path.paths = []
            for obj in relevant_msg.objects:
                obj_pose = obj.pose
                try:
                    path_list_resp = self.get_path_handle(obj_pose)
                    for path in path_list_resp.paths.paths:
                        future_path.paths.append(path)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
            self.path_pub.publish(future_path)
            self.viz_path_list(future_path.paths)

            # ----------------------
            # predict future trajectory for relevant cars, incorporating speed information
            # ----------------------
            predicted_traj = PathArray()
            predicted_traj.header.stamp = rospy.Time.now()
            predicted_traj.header.frame_id = 'map'
            # includes all the predicted trajectories for
            # all the relevant objects
            for obj in relevant_msg.objects:
                speed = np.linalg.norm([obj.twist.linear.x, obj.twist.linear.y])
                waypoint_dist = speed * self.time_step     # predicted distance between waypoints
                obj_pose = obj.pose
                try:
                    path_list_resp = self.get_path_handle(obj_pose)
                    for path in path_list_resp.paths.paths:
                        # path is a ROS path message now
                        traj = self.interpolate_path(path, waypoint_dist)
                        predicted_traj.paths.append(traj)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e

            self.predict_trajectory_pub.publish(predicted_traj)
            self.viz_predicted_traj(predicted_traj.paths)
            # ----------------------
            # publish vehicle marker array for visualization
            # ----------------------
            self.viz_cars(relevant_msg, header)

    def interpolate_path(self, path, distance):
        '''
        path is a ROS path message, interpolate the path so that the waypoits distance
        are *distance*
        return ros path message
        '''
        x_list = []
        y_list = []
        dist_list = []
        for pose_stamped in path.poses:
            x_list.append(pose_stamped.pose.position.x)
            y_list.append(pose_stamped.pose.position.y)
            if len(x_list) == 1:
                dist_list.append(0)
            else:
                dist_incrment = math.sqrt((x_list[-1] - x_list[-2])**2 + (y_list[-1] - y_list[-2])**2)
                dist_list.append(dist_incrment + dist_list[-1])

        dist_interp = [distance * i for i in range(self.steps)]
        x_interp = np.interp(dist_interp, dist_list, x_list)
        y_interp = np.interp(dist_interp, dist_list, y_list)
        interp_traj = Path()
        interp_traj.header = path.header
        for k in range(len(x_interp)):
            pose_stamped = PoseStamped()
            pose_stamped.header = path.header
            pose_stamped.pose.position.x = x_interp[k]
            pose_stamped.pose.position.y = y_interp[k]
            interp_traj.poses.append(pose_stamped)
        return interp_traj

    def viz_predicted_traj(self, path_list):
        viz_msg = MarkerArray()
        count = 0
        for path in path_list:
            # path is a nav_msgs Path message
            msg = Marker()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'map'
            msg.type = Marker.CUBE_LIST
            msg.scale.x = 0.5
            msg.scale.y = 0.5
            msg.scale.z = 0.5
            msg.color.a = 1
            msg.color.b = 1
            msg.id = count
            msg.lifetime = rospy.Duration(0.5)
            msg.points = []
            for pose_stamped in path.poses:
                msg.points.append(pose_stamped.pose.position)
            viz_msg.markers.append(msg)
            count += 1
        self.path_viz_pub.publish(viz_msg)

    def viz_path_list(self, path_list):
        viz_msg = MarkerArray()
        count = 0
        for path in path_list:
            # path is a nav_msgs Path message
            msg = Marker()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'map'
            msg.type = Marker.CUBE_LIST
            msg.scale.x = 0.5
            msg.scale.y = 0.5
            msg.scale.z = 0.5
            msg.color.a = 1
            msg.color.r = 1
            msg.lifetime = rospy.Duration(0.5)
            msg.id = count
            msg.points = []
            for pose_stamped in path.poses:
                msg.points.append(pose_stamped.pose.position)
            viz_msg.markers.append(msg)
            count += 1
        self.path_viz_pub.publish(viz_msg)

    def viz_cars(self, relevant_msg, header):
        marker_msg = MarkerArray()
        count = 0
        for obj in relevant_msg.objects:
            mk = Marker()
            mk.header = header
            mk.type = Marker.CUBE
            mk.scale.x = 3.7
            mk.scale.y = 1.6
            mk.scale.z = 0.5
            mk.color.a = 1
            mk.color.r = 1
            mk.color.g = 0
            mk.color.b = 0
            mk.pose = obj.pose
            mk.id = count
            mk.lifetime = rospy.Duration(0.05)
            marker_msg.markers.append(mk)
            count += 1
        self.marker_pub.publish(marker_msg)

if __name__ == '__main__':
    try:
        relevant_filter = RelevantCarFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
