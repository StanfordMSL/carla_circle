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

try:
    sys.path.append(glob.glob('**/*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

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

        waypoints = world.get_map().get_topology()


        map = defaultdict(set)
        waypoints_Set = set()

        fig = plt.figure()
        for first_pt, second_pt in waypoints:
            pt1 = (round(first_pt.transform.location.x, 2), round(first_pt.transform.location.y, 2))
            pt2 = (round(second_pt.transform.location.x, 2), round(second_pt.transform.location.y, 2))
            if np.linalg.norm(pt1) < 60 and np.linalg.norm(pt2) < 60:
                waypoints_Set.add(pt1)
                waypoints_Set.add(pt2)
                key = ' '.join(str(e) for e in pt1)
                map[key].add(pt2)
                plt.plot([first_pt.transform.location.x, second_pt.transform.location.x], \
                        [first_pt.transform.location.y, second_pt.transform.location.y])
        print("this is the whole list", map)
        print("these are all the waypoints", len(waypoints_Set))
        print("----------------")
        print(np.asarray(list(waypoints_Set)))
        waypoint_tree = KDTree(np.asarray(list(waypoints_Set)))
        plt.grid()
        plt.show()


        def test_waypoint(position):
            q = np.reshape(np.asarray(position), newshape=(2,))
            _, idx = waypoint_tree.query(q)
            next_waypoint = waypoint_tree.data[idx, :].tolist()
            distance = 0
            available_path = []    # this should be a list of available routes
            path = []
            while distance < 40:
                key = ' '.join(str(e) for e in next_waypoint)
                next_points = map[key]
                
                distance += increment


    finally:

        print('\ndestroying %d actors' % len(actor_list))



if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
