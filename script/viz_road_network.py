#!/usr/bin/env python

# author: mingyuw@stanford.edu

'''
this class is to visualize the road network by publishing visualization markers
'''
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
import carla



class RoadGeometryVisualization(object):
    def __init__(self,carla_world, res=2.0):
        rospy.init_node("network_viz", anonymous=True)
        self.world = carla_world
        self.map = carla_world.get_map()

        waypoints =self.map.generate_waypoints(res)
        wp_mat = []
        for wyp in waypoints:
            wp_mat.append([wyp.transform.location.x, -wyp.transform.location.y])
        self.waypoints_mat = np.asarray(wp_mat)

        self.pub = rospy.Publisher("/carla/viz/road_geometry", Marker, queue_size=10)

        self.run()
    def run(self):
        mk = Marker()
        mk.header.stamp = rospy.Time.now()
        mk.header.frame_id = "map"
        mk.type = Marker.CUBE_LIST
        mk.scale.x = 0.4
        mk.scale.y = 0.4
        mk.scale.z = 0.4
        mk.color.a = 1
        for i in range(self.waypoints_mat.shape[0]):
            mk.points.append(Point(self.waypoints_mat[i,0], self.waypoints_mat[i,1], 0))

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub.publish(mk)
            rate.sleep()

if __name__=="__main__":
    host = rospy.get_param("/carla/host", "127.0.0.1")
    port = rospy.get_param("/carla/port", 2000)

    rospy.loginfo("Trying to connect to {host}:{port}".format(
        host=host, port=port))

    try:
        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(2)

        carla_world = carla_client.get_world()

        rospy.loginfo("Connected to Carla.")

        road_viz = RoadGeometryVisualization(carla_world)

        rospy.spin()
        del road_viz
    finally:
        rospy.loginfo("Done")
