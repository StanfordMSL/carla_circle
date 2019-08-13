#!/usr/bin/env python
import rospy
from visualization_classes import DesiredWaypointsVisualization, RoadGeometryVisualization, RoadNetworkVisualization
from visualization_classes import egoTrackVisualizer, oppTrackVisualizer

if __name__ == "__main__":
    try:
        rospy.init_node("viz_pub", anonymous=True)
        waypointsviz = DesiredWaypointsVisualization()
        road_viz = RoadGeometryVisualization()
        road_topo_viz = RoadNetworkVisualization()
        ego_track_viz = egoTrackVisualizer()
        opp_track_viz = oppTrackVisualizer()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
