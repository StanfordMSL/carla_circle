#!/usr/bin/env python
import rospy
from visualization_classes import DesiredWaypointsVisualization, RoadGeometryVisualization


if __name__ == "__main__":
    try:
        rospy.init_node("viz_pub", anonymous=True)
        waypointsviz = DesiredWaypointsVisualization()
        road_viz = RoadGeometryVisualization()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
