#!/usr/bin/env python
import rospy

from visualization_classes import DesiredWaypointsVisualization
from visualization_classes import egoTrackVisualizer
from visualization_classes import oppTrackVisualizer
from visualization_classes import PredictWaypointsVisualization
# from visualization_classes import RoadGeometryVisualization
from visualization_classes import RoadNetworkVisualization

import carla

if __name__ == "__main__":
    try:
        rospy.init_node("viz_pub", anonymous=True)
        ns = rospy.get_namespace()
        rospy.loginfo_once(
            "Initializing visualization node for namespace '{}'".format(ns)
        )

        # Get Carla server information
        host = rospy.get_param("/carla/host")
        port = rospy.get_param("/carla/port")
        carla_client = carla.Client(host, port)
        carla_world = carla_client.get_world()

        waypointsviz = DesiredWaypointsVisualization()
        # road_viz = RoadGeometryVisualization()
        road_topo_viz = RoadNetworkVisualization(carla_world)
        ego_track_viz = egoTrackVisualizer()
        opp_track_viz = oppTrackVisualizer()
        # pre_waypoints_viz = PredictWaypointsVisualization()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
