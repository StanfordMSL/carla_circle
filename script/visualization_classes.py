#!/usr/bin/env python
# author: mingyuw@stanford.edu

# includes all the class need for rviz visualization, note that the RoadGeometryVisualization
# is deprecated. should use the viz_road_network script for road visualization
# maybe combine them together???


import rospy
import tf
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import MultiDOFJointState
from trajectory_msgs.msg import MultiDOFJointTrajectory

import numpy as np


class DesiredWaypointsVisualization:
    def __init__(self):
        rospy.Subscriber("/MSLcar0/command/trajectory", MultiDOFJointTrajectory, self.des_traj_cb)
        self.waypoints_pub = rospy.Publisher("/carla/viz/des_waypoints", Marker, queue_size=10)

    def des_traj_cb(self, msg):
        mk = Marker()
        mk.header.stamp = rospy.Time.now()
        mk.header.frame_id = "map"
        mk.type = Marker.CUBE_LIST
        mk.scale.x = 1.3
        mk.scale.y = 1.3
        mk.scale.z = 1.3
        mk.color.a = 1.0
        mk.color.b = 1
        for pt in msg.points:
            mk.points.append(Point(pt.transforms[0].translation.x, pt.transforms[0].translation.y, 0))
        self.waypoints_pub.publish(mk)



class egoTrackVisualizer(object):
    def __init__(self):
        rospy.Subscriber("/MSLcar0/mpc/ego_track_information", Path, self.ego_track_cb)
        self.area_pub = rospy.Publisher("/carla/viz/ego_track", MarkerArray, queue_size=10)

    def ego_track_cb(self, msg):
        mk_ar = MarkerArray()
        for i in range(len(msg.poses)-2):
            pos_x = msg.poses[i].pose.position.x
            pos_y = msg.poses[i].pose.position.y
            pos_x_n = msg.poses[i+1].pose.position.x
            pos_y_n = msg.poses[i+1].pose.position.y
            yaw = np.arctan2(pos_y_n - pos_y, pos_x_n - pos_x)

            mk = Marker()
            mk.id = i
            mk.header.stamp = rospy.Time.now()
            mk.header.frame_id = "map"
            mk.type = Marker.CUBE
            mk.scale.x = 4
            mk.scale.y = msg.poses[-1].pose.position.x * 2
            mk.scale.z = 1.3
            mk.color.a = 0.2
            mk.color.g = 255
            mk.pose.position.x = pos_x
            mk.pose.position.y = pos_y
            quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
            mk.pose.orientation.x = quat[0]
            mk.pose.orientation.y = quat[1]
            mk.pose.orientation.z = quat[2]
            mk.pose.orientation.w = quat[3]
            mk_ar.markers.append(mk)

        self.area_pub.publish(mk_ar)


class oppTrackVisualizer(object):
    def __init__(self):
        rospy.Subscriber("/MSLcar0/mpc/ado_track_information", Path, self.track_cb)
        self.area_pub = rospy.Publisher("/carla/viz/opp_track", MarkerArray, queue_size=10)

    def track_cb(self, msg):
        mk_ar = MarkerArray()
        for i in range(len(msg.poses)-2):
            pos_x = msg.poses[i].pose.position.x
            pos_y = msg.poses[i].pose.position.y
            pos_x_n = msg.poses[i+1].pose.position.x
            pos_y_n = msg.poses[i+1].pose.position.y
            yaw = np.arctan2(pos_y_n - pos_y, pos_x_n - pos_x)

            mk = Marker()
            mk.id = i
            mk.header.stamp = rospy.Time.now()
            mk.header.frame_id = "map"
            mk.type = Marker.CUBE
            mk.scale.x = 4
            mk.scale.y = msg.poses[-1].pose.position.x * 2
            mk.scale.z = 1.3
            mk.color.a = 0.2
            mk.color.r = 255
            mk.pose.position.x = pos_x
            mk.pose.position.y = pos_y
            quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
            mk.pose.orientation.x = quat[0]
            mk.pose.orientation.y = quat[1]
            mk.pose.orientation.z = quat[2]
            mk.pose.orientation.w = quat[3]
            mk_ar.markers.append(mk)

        self.area_pub.publish(mk_ar)



class RoadNetworkVisualization(object):
    def __init__(self):
        self.road_topo_pub = rospy.Publisher("/carla/viz/road_topo", Marker, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(5), self.timer_cb)
        self.road_topo = [[-2.74, -19.43, -26.26, -7.96], [-14.31, -13.7, -19.69, -4.97], [-16.79, -16.18, -23.05, -5.94], [-12.61, -15.27, -14.31, -13.7], [-14.81, -17.99, -16.79, -16.18], [-2.74, -19.43, -12.61, -15.27], [-3.15, -22.91, -14.81, -17.99], [-8.4, -26.79, -14.81, -17.99], [-14.81, -17.99, -16.79, -16.18], [-16.79, -16.18, -26.26, -7.96], [19.42, 12.53, 19.44, 12.51], [16.47, 10.65, 16.48, 10.64], [19.44, 12.51, 21.85, 7.35], [16.48, 10.64, 18.52, 6.27], [21.85, 7.35, 22.91, 1.72], [18.52, 6.27, 19.41, 1.51], [1.71, 23.72, 8.07, 22.15], [1.37, 20.23, 6.75, 18.9], [8.07, 22.15, 13.71, 18.83], [6.75, 18.9, 11.52, 16.1], [13.71, 18.83, 13.72, 18.82], [11.52, 16.1, 11.53, 16.09], [44.65, -7.81, 43.65, -7.83], [44.6, -4.31, 43.6, -4.33], [43.65, -7.83, 42.65, -7.84], [43.6, -4.33, 42.6, -4.34], [43.48, 3.67, 44.48, 3.68], [43.42, 7.17, 44.42, 7.18], [42.48, 3.65, 43.48, 3.67], [42.42, 7.15, 43.42, 7.17], [14.19, -12.82, 6.32, -25.3], [16.73, -15.22, 9.64, -26.38], [15.33, -11.5, 14.19, -12.82], [18.08, -13.67, 16.73, -15.22], [19.3, -1.96, 15.33, -11.5], [22.78, -2.38, 18.08, -13.67], [25.89, -8.32, 18.08, -13.67], [25.34, -4.86, 15.33, -11.5], [18.08, -13.67, 16.73, -15.22], [15.33, -11.5, 14.19, -12.82], [16.73, -15.22, 9.64, -26.38], [14.19, -12.82, 6.32, -25.3], [25.89, -8.32, 18.08, -13.67], [25.34, -4.86, 15.33, -11.5], [18.08, -13.67, 16.73, -15.22], [15.33, -11.5, 14.19, -12.82], [16.73, -15.22, 4.2, -22.57], [14.19, -12.82, 3.48, -19.15], [-8.4, -26.79, -14.81, -17.99], [-5.07, -25.7, -12.61, -15.27], [-14.81, -17.99, -16.79, -16.18], [-12.61, -15.27, -14.31, -13.7], [-16.79, -16.18, -23.05, -5.94], [-14.31, -13.7, -19.69, -4.97], [-67.31, -2.96, -67.34, -2.96], [-67.36, 0.54, -67.32, 0.54], [-67.31, -2.96, -67.34, -2.96], [-74.31, 21.49, -74.31, 18.95], [-74.31, 18.95, -74.33, 13.87], [-74.33, 13.87, -74.33, 11.33], [-74.33, 11.33, -67.32, 0.54], [-74.39, -8.12, -74.39, -8.15], [-74.39, -8.06, -74.39, -8.12], [-74.39, -8.02, -74.39, -8.06], [-67.34, -2.96, -74.39, -8.02], [-67.31, -2.96, -67.34, -2.96], [-42.35, -2.84, -67.31, -2.96], [-42.35, -2.84, -67.31, -2.96], [-42.35, -2.84, -67.31, -2.96], [-42.35, -2.84, -67.31, -2.96], [-67.32, 0.54, -42.37, 0.66], [-6.39, -44.17, -6.42, -43.18], [-2.89, -44.09, -2.92, -43.09], [-6.42, -43.18, -6.45, -42.19], [-2.92, -43.09, -2.95, -42.07], [7.56, -41.86, 7.58, -42.83], [4.06, -41.89, 4.08, -42.92], [7.58, -42.83, 7.6, -43.83], [4.08, -42.92, 4.1, -43.92], [25.68, 4.02, 42.48, 3.65], [26.51, 7.43, 42.42, 7.15], [-23.05, -5.94, -23.95, 0.38], [-19.69, -4.97, -20.45, 0.38], [-0.5, -23.07, -3.15, -22.91], [-0.5, -23.07, -3.15, -22.91], [-0.5, -19.57, -2.74, -19.43], [-0.5, -19.57, -2.74, -19.43], [4.2, -22.57, -0.5, -23.07], [3.48, -19.15, -0.5, -19.57], [22.95, 0.38, 22.78, -2.38], [22.95, 0.38, 22.78, -2.38], [19.45, 0.38, 19.3, -1.96], [19.45, 0.38, 19.3, -1.96], [42.6, -4.34, 25.34, -4.86], [42.6, -4.34, 25.34, -4.86], [42.65, -7.84, 25.89, -8.32], [42.65, -7.84, 25.89, -8.32], [-21.25, 11.11, -17.09, 16.64], [-21.25, 11.11, -17.09, 16.64], [-18.17, 9.45, -14.61, 14.17], [-18.17, 9.45, -14.61, 14.17], [-21.25, 11.11, -21.25, 11.11], [-18.17, 9.45, -18.17, 9.45], [-17.09, 16.64, -17.09, 16.64], [-14.61, 14.17, -14.61, 14.17], [-17.09, 16.64, -11.86, 20.78], [-14.61, 14.17, -10.11, 17.75], [-11.86, 20.78, -5.61, 23.24], [-10.11, 17.75, -4.83, 19.83], [-23.95, 0.38, -23.87, 2.26], [-20.45, 0.38, -20.38, 1.97], [22.91, 1.72, 22.95, 0.38], [19.41, 1.51, 19.45, 0.38], [13.72, 18.82, 19.42, 12.53], [13.72, 18.82, 19.42, 12.53], [11.53, 16.09, 16.47, 10.65], [11.53, 16.09, 16.47, 10.65], [-27.05, 3.91, -21.25, 11.11], [-0.5, 23.83, 1.71, 23.72], [-0.5, 20.33, 1.37, 20.23], [71.36, -7.41, 44.65, -7.81], [71.31, -3.92, 44.6, -4.31], [44.48, 3.68, 71.19, 4.08], [44.42, 7.18, 71.14, 7.58], [-3.15, -22.91, -14.81, -17.99], [-14.81, -17.99, -16.79, -16.18], [-16.79, -16.18, -26.26, -7.96], [-23.87, 2.26, -23.01, 6.81], [-20.38, 1.97, -19.66, 5.82], [-23.01, 6.81, -21.25, 11.11], [-19.66, 5.82, -18.17, 9.45], [6.32, -25.3, 4.06, -41.89], [9.64, -26.38, 7.56, -41.86], [-17.09, 16.64, -17.08, 16.64], [-14.61, 14.17, -14.61, 14.17], [-17.08, 16.64, -11.75, 26.6], [-14.61, 14.17, -8.32, 25.91], [-8.32, 25.91, -6.59, 40.58], [-11.75, 26.6, -10.09, 40.65], [-5.61, 23.24, -0.5, 23.83], [-4.83, 19.83, -0.5, 20.33], [-42.37, 0.66, -41.37, 0.67], [-41.37, 0.67, -40.41, 0.69], [-41.35, -2.83, -42.35, -2.84], [-40.35, -2.82, -41.35, -2.83], [-6.45, -42.19, -8.4, -26.79], [-6.45, -42.19, -8.4, -26.79], [-2.95, -42.07, -5.07, -25.7], [-26.26, -7.96, -40.35, -2.82], [14.19, -12.82, 3.48, -19.15], [16.73, -15.22, 4.2, -22.57], [15.33, -11.5, 14.19, -12.82], [18.08, -13.67, 16.73, -15.22], [19.3, -1.96, 15.33, -11.5], [22.78, -2.38, 18.08, -13.67], [-10.09, 40.65, -10.08, 41.63], [-6.59, 40.58, -6.58, 41.61], [-10.08, 41.63, -10.07, 42.63], [-6.58, 41.61, -6.57, 42.61], [4.93, 40.58, 7.33, 27.86], [1.43, 40.51, 4.03, 26.7], [4.93, 42.53, 4.92, 41.53], [1.43, 42.56, 1.42, 41.56], [4.92, 41.53, 4.93, 40.58], [1.42, 41.56, 1.43, 40.51], [11.52, 16.1, 11.53, 16.09], [13.71, 18.83, 13.72, 18.82], [4.03, 26.7, 11.52, 16.1], [7.33, 27.86, 13.71, 18.83], [-77.89, -8.11, -77.89, -8.14], [-74.39, -8.12, -74.39, -8.15], [-77.89, -8.05, -77.89, -8.11], [-74.39, -8.06, -74.39, -8.12], [-77.89, -8.01, -77.89, -8.05], [-74.39, -8.02, -74.39, -8.06], [-77.83, 11.34, -77.89, -8.01], [-74.33, 11.33, -74.39, -8.02], [-77.83, 13.88, -77.83, 11.34], [-74.33, 13.87, -74.33, 11.33], [-74.31, 18.95, -74.33, 13.87], [-74.31, 21.49, -74.31, 18.95], [-40.41, 0.69, -27.05, 3.91], [78.88, -3.8, 71.31, -3.92], [78.92, -3.8, 78.88, -3.8], [19.42, 12.53, 19.44, 12.51], [16.47, 10.65, 16.48, 10.64], [19.44, 12.51, 26.51, 7.43], [16.48, 10.64, 25.68, 4.02], [71.14, 7.58, 78.71, 7.7], [71.19, 4.08, 78.76, 4.2], [78.71, 7.7, 78.75, 7.7], [78.76, 4.2, 78.8, 4.2], [-77.89, -8.11, -77.89, -8.14], [-77.89, -8.05, -77.89, -8.11], [-67.31, -2.96, -67.34, -2.96], [77.38, -11.67, 71.36, -7.41]]
    def timer_cb(self, event):
        mk = Marker()
        mk.header.stamp = rospy.Time.now()
        mk.header.frame_id = "map"
        mk.type = Marker.LINE_LIST
        mk.scale.x = 0.4
        mk.color.a = 1
        mk.color.r = 0
        mk.color.b = 0
        mk.color.g = 0
        for i in range(len(self.road_topo)):
            mk.points.append(Point(self.road_topo[i][0], -self.road_topo[i][1], 0))
            mk.points.append(Point(self.road_topo[i][2], -self.road_topo[i][3], 0))
        self.road_topo_pub.publish(mk)



class RoadGeometryVisualization(object):
    def __init__(self):
        self.road_waypoint_pub = rospy.Publisher("/carla/viz/road_geometry", Marker, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(5), self.timer_cb)
        self.road = np.array([[-20.42748451,   0.4571287 ],
               [-23.9241581 ,   0.60972029],
               [-47.35092545,   2.86082792],
               [-42.3509903 ,   2.83511853],
               [-42.3509903 ,   2.83511853],
               [-39.01713562,   2.85658216],
               [-34.47013092,   3.61058998],
               [-30.19406891,   5.33078194],
               [-19.68591499,   4.96807814],
               [-19.68591499,   4.96807814],
               [-23.04741478,   5.94292402],
               [-23.04741478,   5.94292402],
               [-19.46022987,   5.69247198],
               [-22.78059196,   6.7993598 ],
               [-26.26272392,   7.95565557],
               [-24.60193062,   9.23449326],
               [-24.42213249,   9.37671947],
               [-24.22912598,   9.48363209],
               [-17.89901733,   9.17587948],
               [-20.93480301,  10.91772175],
               [-17.50848961,   9.82639217],
               [-20.47308922,  11.68681049],
               [-20.83768463,  12.37615871],
               [-20.66598511,  12.52806377],
               [-20.22568321,  12.35449028],
               [-15.16808128,  12.84201908],
               [-17.70607185,  15.25212383],
               [-17.25204849,  15.7202301 ],
               [-14.6191864 ,  13.39997959],
               [-17.0940609 ,  15.87485409],
               [-17.08897209,  15.88135529],
               [-16.10562515,  15.05532646],
               [-11.70096111,  15.96021652],
               [-13.74266911,  18.8030014 ],
               [-13.29318905,  19.25135422],
               [-13.29318905,  19.25135422],
               [-11.95254993,  17.31732559],
               [-13.21449184,  19.17110252],
               [-10.94542217,  16.65559006],
               [-10.35369778,  22.70158768],
               [-10.35369778,  22.70158768],
               [ -7.69682646,  18.16600418],
               [ -9.00866127,  21.41085815],
               [ -7.51875401,  18.74925995],
               [ -8.40843868,  21.64366913],
               [ -7.41795301,  20.79596329],
               [ -8.39905739,  26.79109955],
               [ -8.39905739,  26.79109955],
               [ -8.12712288,  27.70678329],
               [ -7.29435921,  32.41522217],
               [ -5.07233191,  25.7034874 ],
               [ -6.74423122,  37.29356003],
               [ -4.74600315,  26.80233383],
               [ -3.28474951,  19.36273384],
               [ -3.79235268,  22.82573128],
               [ -6.44617271,  42.19375992],
               [ -6.39222145,  44.17497253],
               [ -6.39222145,  44.17497253],
               [ -2.74491978,  19.43405724],
               [ -3.15410638,  22.91005707],
               [ -6.26887941,  49.17345047],
               [ -3.82766557,  31.93352318],
               [ -3.25732422,  36.99110794],
               [ -2.94831371,  42.07135391],
               [ -2.89328647,  44.08863449],
               [ -2.89328647,  44.08863449],
               [ -2.76994443,  49.08711243],
               [ -0.5       ,  19.56574249],
               [ -0.5       ,  23.06574249],
               [  4.2279253 ,  48.91443253],
               [  4.10458326,  43.91595459],
               [  4.06099606,  41.89071655],
               [  4.06099606,  41.89071655],
               [  4.23421717,  36.74264908],
               [  4.85516453,  31.62923241],
               [  7.72686052,  48.82809448],
               [  7.60351801,  43.82961655],
               [  7.56082249,  41.85582733],
               [  7.56082249,  41.85582733],
               [  3.47695994,  19.14937592],
               [  3.47695994,  19.14937592],
               [  4.20187902,  22.5734787 ],
               [  4.20187902,  22.5734787 ],
               [  7.72378206,  37.01271439],
               [  5.9364562 ,  26.56034851],
               [  6.31678438,  25.2971611 ],
               [  8.30794621,  32.20220566],
               [  4.96018791,  18.77303314],
               [  5.95546913,  22.12854004],
               [  7.07946205,  23.22137642],
               [  9.30949974,  27.49446487],
               [  9.64485455,  26.38064957],
               [  8.40548801,  20.41840172],
               [ 10.31734467,  24.55032921],
               [  7.804811  ,  17.67686272],
               [  9.31860924,  20.83255959],
               [  9.52660561,  18.51218605],
               [ 11.4865675 ,  22.07880974],
               [  9.15703869,  16.96056557],
               [ 10.91732121,  19.98569679],
               [ 12.47511101,  20.39800453],
               [ 11.33187675,  15.99107265],
               [ 11.66191673,  15.22302341],
               [ 14.06690788,  18.1750164 ],
               [ 13.87878609,  17.93143463],
               [ 12.77538872,  14.31571198],
               [ 12.80649376,  14.20737457],
               [ 15.23199654,  16.73065376],
               [ 15.33972359,  16.69776726],
               [ 17.36979866,  14.52204037],
               [ 14.72947884,  12.22449589],
               [ 17.65885735,  14.18320084],
               [ 14.97397041,  11.93789864],
               [ 18.73151398,  12.83734417],
               [ 18.73151398,  12.83734417],
               [ 15.98228741,  10.67129993],
               [ 15.98228741,  10.67129993],
               [ 20.4071312 ,  10.0514307 ],
               [ 17.29852676,   8.44315052],
               [ 20.60790062,   9.65386581],
               [ 17.46833992,   8.10688019],
               [ 21.89564705,   9.81691551],
               [ 21.89564705,   9.81691551],
               [ 20.03248024,   6.85404444],
               [ 20.03248024,   6.85404444],
               [ 25.88729477,   8.31706429],
               [ 25.88729477,   8.31706429],
               [ 25.88729477,   8.31706429],
               [ 30.65486908,   8.07717514],
               [ 22.30157471,   4.98953152],
               [ 35.63582993,   7.95423794],
               [ 18.90089035,   4.16167641],
               [ 22.40267181,   4.55577374],
               [ 18.9864006 ,   3.79479384],
               [ 40.62688828,   7.87409878],
               [ 25.33833313,   4.86038351],
               [ 25.33833313,   4.86038351],
               [ 25.33833313,   4.86038351],
               [ 44.64804459,   7.81403208],
               [ 46.36538696,   7.78837919],
               [ 30.55101776,   4.57871628],
               [ 35.56696701,   4.45491505],
               [ 40.57461166,   4.37448883],
               [ 44.59576797,   4.31442261],
               [ 46.31311035,   4.28876972],
               [ 22.94574356,  -0.38      ],
               [ 19.44574356,  -0.38      ],
               [ 22.90612793,  -1.7205472 ],
               [ 19.41223526,  -1.51386642],
               [ 46.19362259,  -3.71033788],
               [ 44.47628021,  -3.68468499],
               [ 42.47650528,  -3.65481305],
               [ 37.46292114,  -3.59585571],
               [ 22.85946465,  -2.35733652],
               [ 19.37276649,  -2.05247784],
               [ 32.44545364,  -3.58598757],
               [ 27.22084808,  -3.73377514],
               [ 46.14134979,  -7.20994759],
               [ 44.42400742,  -7.1842947 ],
               [ 25.36094666,  -4.10636902],
               [ 42.42422867,  -7.15442276],
               [ 37.4385376 ,  -7.09577084],
               [ 32.45606613,  -7.08597136],
               [ 27.69163895,  -7.20196724],
               [ 26.26242828,  -7.48828125],
               [ 20.29287529,  -6.53933001],
               [ 21.7510891 ,  -7.6472559 ],
               [ 18.43527794,  -6.52681637],
               [ 22.36795235,  -9.35785007],
               [ 19.42364502, -12.52524185],
               [ 19.42364502, -12.52524185],
               [ 16.46667099, -10.65273094],
               [ 16.46667099, -10.65273094],
               [ 17.5301609 , -15.09572601],
               [ 14.86511707, -12.82690811],
               [ 13.72380257, -18.81715584],
               [ 13.72380257, -18.81715584],
               [ 11.53082657, -16.08936119],
               [ 11.53082657, -16.08936119],
               [ 11.83800697, -20.18021202],
               [  9.93577766, -17.2422657 ],
               [ 10.49225998, -22.07882118],
               [  7.73997688, -19.91666222],
               [  8.08978748, -25.98768425],
               [  6.98373795, -22.55672264],
               [  5.82992172, -19.25237465],
               [  6.32819557, -31.15797615],
               [  4.91768026, -24.50858498],
               [  5.32434034, -35.82322311],
               [  4.92610407, -40.57860184],
               [  4.92621231, -42.53359985],
               [  4.92621231, -42.53359985],
               [  4.95784903, -47.53350067],
               [  2.96079636, -30.20371056],
               [  1.70526278, -23.71837616],
               [  1.3652631 , -20.23492813],
               [  1.86247241, -35.30798721],
               [  1.4267596 , -40.5108757 ],
               [  1.42628241, -42.55574417],
               [  1.42628241, -42.55574417],
               [  1.45791936, -47.55564499],
               [ -0.5       , -23.82574272],
               [ -0.5       , -20.32574272],
               [ -6.54192066, -47.60626221],
               [ -6.57355738, -42.60636139],
               [ -6.59354448, -40.58196259],
               [ -6.59354448, -40.58196259],
               [ -6.87697029, -35.4682579 ],
               [-10.04185009, -47.62841034],
               [-10.07348728, -42.62850952],
               [ -5.61353731, -23.24232483],
               [ -4.82515001, -19.83227539],
               [ -7.51765585, -30.38693428],
               [-10.09282017, -40.65314484],
               [-10.09282017, -40.65314484],
               [-10.36268044, -35.78421402],
               [-10.97269821, -30.94611549],
               [ -8.15767193, -22.49519348],
               [ -6.97703934, -19.20033264],
               [ -9.00142479, -23.30851555],
               [-12.32873535, -24.39433479],
               [-11.2663641 , -18.40783119],
               [-11.06642342, -17.15692139],
               [-12.99246788, -20.07931137],
               [-14.2494421 , -20.23847008],
               [-17.08514214, -16.64377213],
               [-17.08514214, -16.64377213],
               [-14.61026764, -14.16889763],
               [-14.61026764, -14.16889763],
               [-18.15013123, -15.53078461],
               [-15.54367828, -13.19489098],
               [-21.252491  , -11.1050539 ],
               [-18.16772842,  -9.4515028 ],
               [-21.38243484, -10.86264324],
               [-22.78365517,  -7.55016375],
               [-19.46282005,  -6.44469309],
               [-23.77931213,  -6.84694481],
               [-27.04977798,  -3.90632653],
               [-20.38011551,  -1.96617115],
               [-23.86815262,  -2.2552979 ],
               [-31.01132393,  -2.19039512],
               [-35.66234207,  -1.13280225],
               [-20.44574356,  -0.38      ],
               [-40.41106796,  -0.68546134],
               [-23.94574356,  -0.38      ],
               [-42.36898804,  -0.6648351 ],
               [-42.36898804,  -0.6648351 ],
               [-47.36892319,  -0.63912582]])
    def timer_cb(self, event):
        mk = Marker()
        mk.header.stamp = rospy.Time.now()
        mk.header.frame_id = "map"
        mk.type = Marker.CUBE_LIST
        mk.scale.x = 0.4
        mk.scale.y = 0.4
        mk.scale.z = 0.4
        mk.color.a = 1
        mk.color.r = 0
        mk.color.b = 1
        mk.color.g = 0
        for i in range(self.road.shape[0]):
            mk.points.append(Point(self.road[i,0], self.road[i,1], 0))
        self.road_waypoint_pub.publish(mk)
