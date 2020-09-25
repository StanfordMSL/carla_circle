#!/usr/bin/env python
# author: mingyuw@stanford.edu

import rospy
import tf
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from derived_object_msgs.msg import ObjectArray, Object
from std_msgs.msg import Header, ColorRGBA

import numpy as np


class GameAdoCarFilter(object):
    '''
    This filter determines the relevant ado, opponent, object for the ego
    vehicle's game plan. The filter is based on a game theoretic planner model
    for two players. Therefore, there is only one "ado/opponent" object
    considered during planning. The node publishes:
        1. One relevant object with "derived_object_msgs" and "odometry"
        2. A marker of the relevant object for visualization
    '''
    def __init__(self):
        '''
        Initializes the class.

        Note: If self.enable_lane_filter is set to true, the filter will take
        into account the lane in which the ado object is present.
        '''
        rospy.init_node("gameAdoCarFilter", anonymous=True)

        # Initialization of class attribute
        self.ego_odom = Odometry()
        self.nearby_obj = ObjectArray()
        self.ego_ready = False
        self.nearby_ready = False
        self.enable_lane_filter = True

        # Subscribers for ego vehicle odometry and Carla objects
        rolename = rospy.get_param("~rolename")
        rospy.loginfo_once(
            "Initializing ado filter for '{}'".format(rolename)
        )
        rospy.Subscriber(
            "/carla/" + rolename + "/objects",
            ObjectArray,
            self.nearby_car_cb
        )
        rospy.Subscriber(
            "/carla/" + rolename + "/odometry", Odometry,
            self.ego_vehicle_cb
        )

        # Publishers for odometry of ego and opponent
        self.ego_odom_pub = rospy.Publisher(
            "MSLcar0/ground_truth/odometry",
            Odometry,
            queue_size=10
        )
        self.opp_odom_pub = rospy.Publisher(
            "MSLcar1/ground_truth/odometry",
            Odometry,
            queue_size=10
        )

        # Publishers for visualization of relevant objects
        self.marker_pub = rospy.Publisher(
            "/carla/viz/relevant_obj",
            MarkerArray,
            queue_size=10
        )
        self.ado_marker_pub = rospy.Publisher(
            "/carla/viz/game_ado_obj",
            MarkerArray,
            queue_size=10
        )

        # Class timer
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)

    def ego_vehicle_cb(self, msg):
        '''
        This is a callback for the ego vehicle that fills out the ego vehicle's
        odometry information and set the ego_ready variable.

        Parameters
        ----------
        msg : Odometry
            The odometry information of the ego vehicle.
        '''
        self.ego_odom = msg

        if not self.ego_ready:
            self.ego_ready = True

    def nearby_car_cb(self, msg):
        '''
        This is a callback that fills out the nearby objects for the ego
        vehicle and set the nearby_ready variable.

        Parameters
        ----------
        msg : ObjectArray
            An array of all Carla vehicles and pedestrians in the scene,
            excluding the ego vehicle's object.
        '''
        self.nearby_obj = msg

        if not self.nearby_ready:
            self.nearby_ready = True

    def timer_cb(self, event):
        '''
        This is a callback for the class timer, which will be called every
        tick. The callback will filter out objects not relevant to the ego
        vehicle and publish to the ROS topics defined in the class.

        Parameters
        ----------
        event : rospy.TimerEvent
            The timer's tick event.
        '''
        def relevant_filter(pos_x, pos_y, yaw, object):
            '''
            Determines if the object is in the circle and relevant to the
            current ego position.

            Parameters
            ----------
            pos_x : float64
                The x location of the ego vehicle.
            pos_y : float64
                The y location of the ego vehicle.
            yaw : float64
                The yaw, heading, of the ego vehicle.
            object : derived_object_msgs.Object
                The converted Carla object.

            Returns
            -------
            bool, int, float
                A tuple containing the information about the provided object.
                Specifically, whether the object is relevant, its location
                (in front, 1, or behind, 0) and distance [meters], all with
                respect to the ego vehicle.
            '''
            # Position of the object of interest
            car_pos = [object.pose.position.x, object.pose.position.y]

            # If position is too close to circle center, ignore
            if np.linalg.norm(car_pos) <= 5:
                rospy.logdebug("Ado object %d in fountain.", object.id)
                return False, 0, 0

            # Orientation of object
            quat = (
                object.pose.orientation.x,
                object.pose.orientation.y,
                object.pose.orientation.z,
                object.pose.orientation.w
            )
            car_yaw = tf.transformations.euler_from_quaternion(quat)[2]
            ori_car = [np.cos(car_yaw), np.sin(car_yaw)]
            ori_circle = np.negative(car_pos)/np.linalg.norm(car_pos)
            rel_ang = np.dot(ori_circle, ori_car)

            rospy.logdebug("Position of ado: {}".format(car_pos))
            rospy.logdebug("Heading/yaw of ado: {}".format(car_yaw))
            rospy.logdebug("Global orientation: {}".format(ori_car))
            rospy.logdebug("Circle orientation: {}".format(ori_circle))
            rospy.logdebug("Relative heading: {}".format(rel_ang))

            # If object is facing away from circle (exiting), ignore
            if rel_ang <= -0.3:
                rospy.logdebug("Ado object %d exiting the circle.", object.id)
                return False, 0, 0

            # Unit vector representation of ego heading
            ori_ego = [np.cos(yaw), np.sin(yaw)]
            # Relative position of object w.r.t ego vehicle
            ori_rel = [car_pos[0] - pos_x, car_pos[1] - pos_y]
            # Distance from object along the heading of ego car
            distance_tangent = np.dot(ori_rel, ori_ego)
            # Distance from object along normal of the ego car heading
            distance_normal = np.dot(ori_rel, [ori_ego[1], -ori_ego[0]])

            # Object's relative postion is relevant
            if (
                distance_tangent > -15 and
                distance_tangent < 25 and
                abs(distance_normal) < 25
            ):
                if distance_tangent > 0:
                    rospy.logdebug("Ado object %d is in front.", object.id)
                    return True, 1, np.linalg.norm(ori_rel)
                else:
                    rospy.logdebug("Ado object %d is behind.", object.id)
                    return True, 0, np.linalg.norm(ori_rel)

            # Object is in the circle, but not relevant to ego location
            rospy.logdebug("Ado object %d is not relevant.", object.id)
            return False, 0, 0

        def relevant_filter_entrance(pos_x, pos_y, object):
            '''
            Determines if the object is trying to enter the circle and relevant
            to the current ego position. The filter will take into account
            whether the object is ahead of the ego and whether to base its
            relevancy on the object's current lane.

            Parameters
            ----------
            pos_x : float64
                The x location of the ego vehicle.
            pos_y : float64
                The y location of the ego vehicle.
            object : derived_object_msgs.Object
                The converted Carla object.

            Returns
            -------
            bool
                Whether the object is relevant to the ego vehicle.
            '''
            pos_car = [object.pose.position.x, object.pose.position.y]

            if not self.enable_lane_filter:
                if pos_x > 0 and pos_y < 0:
                    if (pos_car[0] > 15 and pos_car[1] > 0):
                        return True
                    return False
                elif pos_x > 0 and pos_y >= 0:
                    if (pos_car[0] < 0 and pos_car[1] > 15):
                        return True
                    return False
                elif pos_x <= 0 and pos_y >= 0:
                    if (pos_car[0] < -20 and pos_car[1] < 0):
                        return True
                    return False
                else:
                    if pos_car[0] > 0 and pos_car[1] < -20:
                        return True
                    return False
            else:  # TODO: Update to use waypoint information
                if pos_x > 0 and pos_y < 0:
                    if (pos_car[0] > 15 and pos_car[1] > 0 and pos_car[1] < 7):
                        return True
                    return False
                elif pos_x > 0 and pos_y >= 0:
                    if (
                        (
                            pos_car[0] < 0 and
                            pos_car[0] > -6 and
                            pos_car[1] > 15
                        ) or
                        (
                            pos_car[0] < -5 and
                            pos_car[0] > -10 and
                            pos_car[1] > 20 and
                            pos_car[1] < 25
                        )
                    ):
                        return True
                    return False
                elif pos_x <= 0 and pos_y >= 0:
                    return False
                else:
                    if (
                        (
                            pos_car[0] > 0 and
                            pos_car[0] < 5 and
                            pos_car[1] < -20
                        ) or
                        (
                            pos_car[0] > 5 and
                            pos_car[0] < 10 and
                            pos_car[1] < -20 and
                            pos_car[1] > -25
                        )
                    ):
                        return True

                    return False

        if self.ego_ready and self.nearby_ready:
            self.ego_odom_pub.publish(self.ego_odom)

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

            ori_quat = (
                self.ego_odom.pose.pose.orientation.x,
                self.ego_odom.pose.pose.orientation.y,
                self.ego_odom.pose.pose.orientation.z,
                self.ego_odom.pose.pose.orientation.w
            )
            ori_euler = tf.transformations.euler_from_quaternion(ori_quat)
            yaw = ori_euler[2]

            # For each object nearby, check if relevant and store.
            for obj in self.nearby_obj.objects:
                bool_rel, rel_pos, dist = relevant_filter(
                    self.ego_odom.pose.pose.position.x,
                    self.ego_odom.pose.pose.position.y,
                    yaw,
                    obj
                )

                if bool_rel:
                    relevant_msg.objects.append(obj)

                    if rel_pos == 1:
                        if dist < ahead_distance:
                            ahead_distance = dist
                            front_closest_obj = obj
                    else:
                        if dist < behind_distance:
                            behind_distance = dist
                            behind_closest_obj = obj

            # Visualization of all relevant objects
            color_relevant = ColorRGBA(0.0, 0.0, 1.0, 0.5)
            self.viz_cars(
                relevant_msg.objects,
                header,
                color_relevant,
                self.marker_pub
            )

            # Determine the current opponent. Default to closest relevant front
            # object, then closest relevant behind object, otherwise create
            # ghost opponent far from the circle.
            if front_closest_obj:
                ado_object = front_closest_obj
            elif behind_closest_obj:
                ado_object = behind_closest_obj
            else:
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

            # Visualization of current opponent
            color_opp = ColorRGBA(1.0, 0.0, 0.0, 1.0)
            self.viz_cars([ado_object], header, color_opp, self.ado_marker_pub)

    def viz_cars(self, obj_list, header, color_rgba, publisher):
        '''
        Publish objects to the provied rviz publisher.

        Parameters
        ----------
        obj_list : [Objects]
            The list of Objects that should be visualized in rviz.
        header : Header
            The Header object containing the current ROS time and frame id.
        color_rgba : ColorRGBA
            The color in which the objects should be visualized.
        publisher : rospy.Publisher
            The ROS Publisher in which to publish the messages.
        '''
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
