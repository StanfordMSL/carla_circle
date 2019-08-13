#!/usr/bin/env python
# author: mingyuw@stanford.edu


# this node filters out vehicles close to the roundabout

import rospy
from derived_object_msgs.msg import ObjectArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

import timeit
import numpy as np

class CarFilter:
    def __init__(self):
        rospy.init_node("perception", anonymous=True)
        rospy.Subscriber("/carla/objects", ObjectArray, self.allcar_cb)
        self.filter_pub = rospy.Publisher("/carla/nearby_obj", ObjectArray, queue_size=10)
        self.viz_pub = rospy.Publisher("/carla/viz/nearby_markers", MarkerArray, queue_size=10)

    def allcar_cb(self, objectarray_msg):
        def valid_car(object):
            pos_x = object.pose.position.x
            pos_y = object.pose.position.y
            dist = np.linalg.norm([pos_x, pos_y])
            return dist < 60

        msg = ObjectArray()
        msg.header = objectarray_msg.header
        msg.objects = [obj for obj in objectarray_msg.objects if valid_car(obj)]
        self.filter_pub.publish(msg)


        viz_msg = MarkerArray()
        count = 0
        for obj in msg.objects:
            mk = Marker()
            mk.header = objectarray_msg.header
            mk.type = Marker.CUBE
            mk.scale.x = 3.7
            mk.scale.y = 1.6
            mk.scale.z = 0.5
            mk.color.a = 1
            mk.color.r = 255
            mk.color.g = 255
            mk.color.b = 0
            mk.pose = obj.pose
            mk.id = count
            mk.lifetime = rospy.Duration(0.05)
            viz_msg.markers.append(mk)
            count += 1
        self.viz_pub.publish(viz_msg)

if __name__ == '__main__':
    try:
        filter = CarFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
