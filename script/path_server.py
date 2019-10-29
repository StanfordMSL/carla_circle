#!/usr/bin/env python
#
# author: mingyuw@stanford.edu

"""
this node allows query next several waypoints from a given position
If there are several possible paths, then all the path are returned
"""
import rospy
import tf
from tf.transformations import euler_from_quaternion

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from carla_circle.msg import PathArray
from carla_circle.srv import GetAvailablePath

import math
import carla


class PathServer(object):
    """
    this class generates available future paths given a carla.location
    in the road network
    """
    WAYPOINT_DISTANCE = 3.0
    PATH_LENGTH = 20      #    10 waypoints, 30m in total

    def __init__(self, carla_world):
        rospy.init_node("path_server", anonymous=True)
        self.world = carla_world
        self.map = carla_world.get_map()

        self.service = rospy.Service('get_path', GetAvailablePath, self.service_cb)

    def service_cb(self, req):

        def get_pose_msg(waypoint, header):
            # this fucntion returns a stamped PoseStamped msg given a carla.Waypoint
            msg = PoseStamped()
            msg.header = header
            msg.pose.position.x = waypoint.transform.location.x
            msg.pose.position.y = -waypoint.transform.location.y
            return msg

        loc = carla.Location()
        loc.x = req.pose.position.x
        loc.y = -req.pose.position.y
        current_wpt = self.map.get_waypoint(loc)

        path_list = []
        path_list.append([current_wpt])
        for i in range(self.PATH_LENGTH):
            new_list = []
            while len(path_list) != 0:
                path = path_list.pop()
                current_wpt = path[-1]
                next_list = current_wpt.next(self.WAYPOINT_DISTANCE)
                for next in next_list:
                    path_new = path[:]
                    path_new.append(next)
                    new_list.append(path_new)
            path_list = new_list[:]

        paths_msg = PathArray()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        paths_msg.header = header
        paths_msg.paths = []
        for path in path_list:
            path_msg = Path()
            path_msg.header = header
            path_msg.poses = [get_pose_msg(wpt, header) for wpt in path]
            paths_msg.paths.append(path_msg)
        return paths_msg

def main():

    host = rospy.get_param("/carla/host", "localhost")
    port = rospy.get_param("/carla/port", 2000)

    rospy.loginfo("Trying to connect to {host}:{port}".format(
        host=host, port=port))

    try:
        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(2)

        carla_world = carla_client.get_world()

        rospy.loginfo("Connected to Carla.")

        path_server = PathServer(carla_world)

        rospy.spin()
        del path_server
        del carla_world
        del carla_client

    finally:
        rospy.loginfo("Done")


if __name__ == "__main__":
    main()
