#!/usr/bin/env python
"""
Stanford MSL Carla Ego Vehicle
"""
from carla_ego_vehicle.carla_ego_vehicle_base import CarlaEgoVehicleBase

import math
import random
import rospy
import time
from tf.transformations import euler_from_quaternion

import carla


class MSLEgoVehicle(CarlaEgoVehicleBase):
    """
    Stanford MSL Carla Ego Vehicle.

    Takes in several parameters to initialize a Carla ROS ego vehicle within
    their ROS bridge interface.
    """

    def __init__(self):  # Overrides base class
        """
        Modifies the init function of the base class to handle more parameters.
        """
        rospy.init_node('ego_vehicle')
        self.world = None
        self.player = None
        self.sensor_actors = []
        self.host = rospy.get_param('/carla/host', '127.0.0.1')
        self.port = rospy.get_param('/carla/port', '2000')
        self._color = rospy.get_param('~color', '0, 0, 0')
        self._rolename = rospy.get_param('~rolename', 'dummy')
        self.actor_filter = rospy.get_param('~vehicle_filter', 'vehicle.*')
        spawn_point = rospy.get_param('~spawn_point', [-3.0, -30.0, 2.0, 90.0])
        self.actor_spawnpoint = carla.Transform(
            carla.Location(x=spawn_point[0], y=spawn_point[1], z=spawn_point[2]),
            carla.Rotation(yaw=spawn_point[3])
        )
        rospy.loginfo('Listening to server %s:%s', self.host, self.port)
        rospy.loginfo('Using vehicle filter: %s', self.actor_filter)
        rospy.loginfo('Rolename: %s', self._rolename)
        rospy.loginfo('Vehicle color: %s', self._color)
        rospy.loginfo('Vehicle initial pose: %s', spawn_point)

    def restart(self):  # Overrides base class
        # Get vehicle blueprint.
        blueprint = random.choice(self.world.get_blueprint_library().filter(self.actor_filter))
        blueprint.set_attribute('role_name', self._rolename)

        if blueprint.has_attribute('color'):
            blueprint.set_attribute('color', self._color)

        # Spawn the vehicle.
        if self.actor_spawnpoint:
            spawn_point = self.actor_spawnpoint

            if self.player is not None:
                self.destroy()
            else:
                self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        else:
            if self.player is not None:
                spawn_point = self.player.get_transform()
                spawn_point.location.z += 2.0
                spawn_point.rotation.roll = 0.0
                spawn_point.rotation.pitch = 0.0
                self.destroy()
                self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            else:
                spawn_points = self.world.get_map().get_spawn_points()
                spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
                self.player = self.world.try_spawn_actor(blueprint, spawn_point)

        if self.player:
            time.sleep(1.0)
            rospy.loginfo(
                "Spawned at x={} y={} z={} yaw={}".format(
                    self.player.get_location().x,
                    self.player.get_location().y,
                    self.player.get_location().z,
                    spawn_point.rotation.yaw
                )
            )
        else:
            self.restart()

        # Set up the sensors
        self.sensor_actors = self.setup_sensors(self.sensors())

def main():
    """
    Main function
    """
    ego_vehicle = MSLEgoVehicle()

    try:
        ego_vehicle.run()
    finally:
        if ego_vehicle is not None:
            ego_vehicle.destroy()


if __name__ == '__main__':
    main()
