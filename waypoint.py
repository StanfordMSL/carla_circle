#!/usr/bin/env python

# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Spawn NPCs into the simulation"""

import glob
import os
import sys
import time

try:
    sys.path.append(glob.glob('**/*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from carla import Transform, Rotation, Location
import argparse
import random
import time
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
from scipy.spatial import KDTree


def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=10,
        type=int,
        help='number of vehicles (default: 10)')
    argparser.add_argument(
        '-d', '--delay',
        metavar='D',
        default=2.0,
        type=float,
        help='delay in seconds between spawns (default: 2.0)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='avoid spawning vehicles prone to accidents')
    args = argparser.parse_args()

    actor_list = []

    try:

        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)
        world = client.get_world()

        map = world.get_map()

        # ------------------------------
        # test teleport vehicle
        # ------------------------------
        # blueprint = random.choice(world.get_blueprint_library().filter('toyota'))
        # blueprint.set_attribute('role_name', 'hero')
        # #        blueprint.set_attribute('role_name', 'ego_vehicle')
        # if blueprint.has_attribute('color'):
        #     # color = random.choice(blueprint.get_attribute('color').recommended_values)
        #     # print("what is color", type(color))
        #     color = "255,255,255"
        #     blueprint.set_attribute('color', color)
        # spawn_point = Transform(Location(x=0.8063, y=20.0, z=2.5), Rotation(pitch=0, yaw=0.0001, roll=0))
        # actors = world.get_actors()
        #
        # vehicle = actors.find(43)
        # # print("get the vehicle", vehicle[-1])
        # print("here is the vehicle location", vehicle.get_location().x, vehicle.get_location().y)
        # waypoint = map.get_waypoint(vehicle.get_location())
        #
        # # Disable physics, in this example we're just teleporting the vehicle.
        # vehicle.set_simulate_physics(False)
        #
        # while True:
        #     # Find next waypoint 2 meters ahead.
        #     waypoint = random.choice(waypoint.next(2.0))
        #     # Teleport the vehicle.
        #     vehicle.set_transform(waypoint.transform)
        #     time.sleep(1.0)



        # -----------------------------------
        # plot round about road network
        # -----------------------------------
        # waypoints = world.get_map().get_topology()
        # dense_waypoint = world.get_map().generate_waypoints(2.0)
        #
        #
        #
        # waypoints_Set = set()
        # #
        # fig = plt.figure()
        # for first_pt, second_pt in waypoints:
        #     pt1 = (round(first_pt.transform.location.x, 2), round(first_pt.transform.location.y, 2))
        #     pt2 = (round(second_pt.transform.location.x, 2), round(second_pt.transform.location.y, 2))
        #     if np.linalg.norm(pt1) < 60 and np.linalg.norm(pt2) < 60:
        #         waypoints_Set.add(pt1)
        #         waypoints_Set.add(pt2)
        #         plt.text(first_pt.transform.location.x, first_pt.transform.location.y, s="l:" + str(first_pt.lane_id))
        #         plt.text(first_pt.transform.location.x, first_pt.transform.location.y+2, s="r:" + str(first_pt.road_id))
        #         print("here is the first point", first_pt.transform.location.x, first_pt.transform.location.y, " its road id", first_pt.road_id)
        #         # plt.text(first_pt.transform.location.x, first_pt.transform.location.y+2, s="lane width:" + str(first_pt.lane_width))
        #         plt.plot([first_pt.transform.location.x, second_pt.transform.location.x], \
        #                 [-first_pt.transform.location.y, -second_pt.transform.location.y])
        # print("this is the whole list", map)
        # print("these are all the waypoints", len(waypoints_Set))
        # print("----------------")
        # print(np.asarray(list(waypoints_Set)))
        # waypoint_tree = KDTree(np.asarray(list(waypoints_Set)))
        # plt.grid()




        # ----------------------------
        # test generate available future path
        # ----------------------------

        # def get_location(waypoint):
        #     return [waypoint.transform.location.x, \
        #             waypoint.transform.location.y]
        #
        # loc = carla.Location()
        # loc.x = 13.6
        # loc.y = 17.8
        # current_wpt = map.get_waypoint(loc)
        #
        # path_list = []
        # path_list.append([current_wpt])
        # for i in range(10):
        #     # print("---expanding the ", i, "th depth")
        #     new_list = []
        #     while len(path_list) != 0:
        #         path = path_list.pop()
        #         current_wpt = path[-1]
        #         next_list = current_wpt.next(3.0)
        #         for next in next_list:
        #             path_new = path[:]
        #             path_new.append(next)
        #             new_list.append(path_new)
        #     path_list = new_list[:]
        #
        # loation_path_list = []
        # for path in path_list:
        #     location_path = [get_location(wyt) for wyt in path]
        #     location_path_array = np.asarray(location_path)
        #     plt.plot(location_path_array[:,0], -location_path_array[:,1])
        #
        #     print("hereis one  path", location_path_array)
        #
        # plt.show()


        # ----------------------------
        # generate dense waypoints for ego car in the circle
        # ----------------------------

        waypoints = map.generate_waypoints(5.0)
        fig = plt.figure()
        ax = fig.add_subplot(111)
        waypoints_list = []
        ang = []
        for wy in waypoints:
            dist = np.linalg.norm([wy.transform.location.x, wy.transform.location.y])
            if dist < 50:
                plt.scatter(wy.transform.location.x, -wy.transform.location.y)
                waypoints_list.append([wy.transform.location.x, -wy.transform.location.y])
                ang.append(np.arctan2(wy.transform.location.y, wy.transform.location.x))
        waypoints_array = np.asarray(waypoints_list)
        sort_index = np.argsort(ang)
        sorted_waypoints = waypoints_array[sort_index, :]
        print("these are the sorted waypoits", sorted_waypoints)
        # plt.plot(sorted_waypoints[:,0], sorted_waypoints[:,1])


        radius = 20
        ang_increment = 2* np.pi / 300
        ang = [i * ang_increment for i in range(300)]
        circle = np.zeros((300, 2))
        circle[:,0] = radius * np.cos(ang) - 0.5
        circle[:,1] = radius * np.sin(ang) - 0.3
        plt.plot(circle[:,0], circle[:,1])
        ticks = np.arange(-40, 40, 5)
        ax.set_xticks(ticks)
        ax.set_yticks(ticks)
        plt.grid()
        plt.axis("equal")
        plt.show()





    finally:

        print('\ndestroying %d actors' % len(actor_list))



if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
