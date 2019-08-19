#!/usr/bin/env python
# author: mingyuw@stanford.edu

"""
this node publishes the following information
** for 2-person game theoretic planner **
there is only one "ado/opponent" vehicle considered by the game theoretic planner
1. one relevant vehicle as derived_object_msgs and odometry information
2. marker of relevant vehicle for visualiation
"""
import rospy
import tf
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from derived_object_msgs.msg import ObjectArray, Object
from std_msgs.msg import Header, ColorRGBA

import numpy as np
import math
import random


class GameAdoCarFilter(object):
    def __init__(self):
        rospy.init_node("gameAdoCarFilter", anonymous=True)

        # -----------------
        # setup attributes for class
        # -------------------
        self.ego_odom = Odometry()
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
        # publishers for ego and opponent for game planner
        self.ego_odom_pub = rospy.Publisher("MSLcar0/ground_truth/odometry", Odometry, queue_size=10)
        self.opp_odom_pub = rospy.Publisher("MSLcar1/ground_truth/odometry", Odometry, queue_size=10)
        self.ado_marker_pub = rospy.Publisher("/carla/viz/game_ado_obj", MarkerArray, queue_size=10)

        # update timer
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)

    def ego_vehicle_cb(self, msg):
        self.ego_odom = msg
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

        def relevant_filter(pos_x, pos_y, yaw, object):
            # return if the object is relevant, if relevant, return its relative position (ahead, behind), and distance
            # 0 means behind, 1 means in front of us

            # get rid of ego vehicle
            car_pos = [object.pose.position.x, object.pose.position.y]    # position of the car we are interested in
            dis = np.linalg.norm([car_pos[0] - pos_x, car_pos[1] - pos_y])
            if dis < 0.5:
                return False, 0, 0

            if np.linalg.norm(car_pos) <= 5:
                return False, 0, 0

            # first filter based on the orientation of car
            # if going out of a circle, then not relevant

            quat = (object.pose.orientation.x,
                        object.pose.orientation.y,
                        object.pose.orientation.z,
                        object.pose.orientation.w)
            car_yaw = tf.transformations.euler_from_quaternion(quat)[2]
            ori_car = [np.cos(car_yaw), np.sin(car_yaw)]
            ori_circle = [-car_pos[0], -car_pos[1]]
            ori_circle = ori_circle/np.linalg.norm(ori_circle)
            rel_ang = np.dot(ori_circle, ori_car)
            # print(" this is the position of car", car_pos)
            # print(" this is yaw angle ", car_yaw)
            # print(" this is ori car ", ori_car)
            # print(" this is ori circle ", ori_circle)
            # print("this is rel ang ", rel_ang)
            if rel_ang <= -0.3:
                return False, 0, 0

            # then, filter based on position
            # according to the position and orientation of ego vehicle, cars not too far behind/ in front are relevant
            ori_ego = [np.cos(yaw), np.sin(yaw)]      # unit vector representation of orientation of the ego car
            ori_rel = [car_pos[0] - pos_x, car_pos[1] - pos_y]       # relative position of obj w.r.t ego car
            distance_tangent = np.dot(ori_rel, ori_ego)      # distance along the orientation of ego car
            distance_normal = np.dot(ori_rel, [ori_ego[1], -ori_ego[0]])
            if distance_tangent > -15 and distance_tangent < 25 and abs(distance_normal) < 25:
                if distance_tangent > 0:
                    return True, 1, np.linalg.norm(ori_rel)
                else:
                    return True, 0, np.linalg.norm(ori_rel)

            return False, 0, 0

        def relevant_filter_entrance(pos_x, pos_y, object):
            # relevant car: cars entering the circle through the entrance in front of us
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
            # ------------------------------
            #  publish ego odometry
            # ------------------------------
            self.ego_odom_pub.publish(self.ego_odom)

            # ------------------------------
            #  publish relevant car objectarray_msg and visualization
            #  also save the car closest to us (in front of us and behind us, respectively)
            # ------------------------------
            ahead_distance = 100
            behind_distance = 100
            front_closest_obj = None
            behind_closest_obj = None



            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'map'
            relevant_msg = ObjectArray()
            relevant_msg.header = header
            relevant_msg.objects = []

            ori_quat = (self.ego_odom.pose.pose.orientation.x,
                        self.ego_odom.pose.pose.orientation.y,
                        self.ego_odom.pose.pose.orientation.z,
                        self.ego_odom.pose.pose.orientation.w)
            ori_euler = tf.transformations.euler_from_quaternion(ori_quat)
            yaw = ori_euler[2]
            for obj in self.nearby_obj.objects:
                bool_rel, rel_pos, dist = relevant_filter(self.ego_odom.pose.pose.position.x, self.ego_odom.pose.pose.position.y, yaw, obj)
                if bool_rel:
                    # the obj is relevant
                    relevant_msg.objects.append(obj)
                    if rel_pos == 1:
                        if dist < ahead_distance:
                            ahead_distance = dist
                            front_closest_obj = obj
                    else:
                        if dist < behind_distance:
                            behind_distance = dist
                            behind_closest_obj = obj

            # relevant_msg.objects = [obj for obj in self.nearby_obj.objects \
            #                 if relevant_filter(self.ego_odom.pose.pose.position.x, self.ego_odom.pose.pose.position.y, yaw, obj)]
            self.relevant_pub.publish(relevant_msg)
            # visualization
            color_relevant = ColorRGBA()
            color_relevant.r = 0
            color_relevant.g = 0
            color_relevant.b = 1
            color_relevant.a = 0.5
            self.viz_cars(relevant_msg.objects, header, color_relevant, self.marker_pub)


            # ----------------------
            # publish only one game opponent car
            # ** if there are multiple relevant cars, then randomly pick one
            # ** if there is no relevant cars, fake one far away
            # ----------------------
            if front_closest_obj:
                ado_object = front_closest_obj
                # ado_object = random.choice(relevant_msg.objects)

            elif behind_closest_obj:
                ado_object = behind_closest_obj

            else:
                # if there is no relevant car as opponent, then make up one
                ado_object = Object()
                ado_object.header.stamp = rospy.Time.now()
                ado_object.header.frame_id = "map"
                ado_object.pose.position.x = -5.5
                ado_object.pose.position.y = 170
                ado_object.pose.position.z = 0
                ado_object.pose.orientation.x = 0
                ado_object.pose.orientation.y = 0
                ado_object.pose.orientation.z = 0
                ado_object.pose.orientation.w = 1
                ado_object.twist.linear.x = 0
                ado_object.twist.linear.y = 0
                ado_object.twist.linear.z = 0

            ado_odom = Odometry()
            ado_odom.header.stamp = rospy.Time.now()
            ado_odom.header.frame_id = "map"
            ado_odom.child_frame_id = "ado"
            ado_odom.pose.pose = ado_object.pose
            ado_odom.twist.twist = ado_object.twist

            self.opp_odom_pub.publish(ado_odom)

            color_opp = ColorRGBA()
            color_opp.r = 1
            color_opp.g = 0
            color_opp.b = 0
            color_opp.a = 1
            self.viz_cars([ado_object], header, color_opp, self.ado_marker_pub)







    def viz_cars(self, obj_list, header, color_rgba, publisher):

        marker_msg = MarkerArray()
        count = 0
        for obj in obj_list:
            mk = Marker()
            mk.header = header
            mk.type = Marker.CUBE
            mk.scale.x = 3.7
            mk.scale.y = 1.6
            mk.scale.z = 0.5
            mk.color = color_rgba
            mk.pose = obj.pose
            mk.id = count
            mk.lifetime = rospy.Duration(1)
            marker_msg.markers.append(mk)
            count += 1
        publisher.publish(marker_msg)

if __name__ == '__main__':
    try:
        game_car_filter = GameAdoCarFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
