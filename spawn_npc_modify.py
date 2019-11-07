#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Spawn NPCs into the simulation"""

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import time
import argparse
import logging
import random
import numpy as np

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
        default=15,
        type=int,
        help='number of vehicles (default: 10)')
    argparser.add_argument(
        '-ni', '--number-each-instance',
        metavar='NI',
        default=3,
        type=int,
        help='number of vehicles spawned at each instance (default: 3)')
    argparser.add_argument(
        '-d', '--delay',
        metavar='D',
        default=5.0,
        type=float,
        help='delay in seconds between spawns (default: 2.0)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='avoid spawning vehicles prone to accidents')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    actor_list = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)

    try:

        # #########################################
        # get all the blueprints according to arguments input
        # #########################################
        world = client.get_world()
        blueprints = world.get_blueprint_library().filter('vehicle.*')

        if args.safe:
            blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
            blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]


        # #########################################
        # get all the spawn points close to the circle
        # #########################################
        spawn_points = world.get_map().get_spawn_points()

        def valid(transform):
            if transform.location.x > 20 and transform.location.x < 50 \
                    and transform.location.y < 0 and transform.location.y > -7:
                return True
            if transform.location.x > 5\
                    and transform.location.y > 20 and transform.location.y < 50:
                return True
            if transform.location.x < 0 and transform.location.x > -20 and \
                    transform.location.y < -20 and transform.location.y > -50:
                return True
            if transform.location.x < -20 and transform.location.x > -50 and \
                    transform.location.y > 0 and transform.location.y < 20:
                return True
            return False
        spawn_points = [spawn_point for spawn_point in spawn_points if valid(spawn_point)]

        number_of_spawn_points = len(spawn_points)

        # #########################################
        # spawn only (num each instance) or the number of available spawn points, whichever is smaller
        # #########################################
        if args.number_each_instance < number_of_spawn_points:
            random.shuffle(spawn_points)
        else:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, args.number_each_instance, number_of_spawn_points)
            args.number_each_instance = number_of_spawn_points

        # if args.number_of_vehicles < number_of_spawn_points:
        #     random.shuffle(spawn_points)
        # elif args.number_of_vehicles > number_of_spawn_points:
        #     msg = 'requested %d vehicles, but could only find %d spawn points'
        #     logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
        #     args.number_of_vehicles = number_of_spawn_points

        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor



        def try_update_actors():
            spawn_number = min(args.number_each_instance, args.number_of_vehicles - len(actor_list))
            # #########################################
            # prepare spawn batch
            # #########################################

            spawn_batch = []
            for n, transform in enumerate(spawn_points):
                if n >= spawn_number:
                    break
                blueprint = random.choice(blueprints)
                if blueprint.has_attribute('color'):
                    color = random.choice(blueprint.get_attribute('color').recommended_values)
                    blueprint.set_attribute('color', color)
                blueprint.set_attribute('role_name', 'autopilot')
                spawn_batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True)))

            # #########################################
            # delete batch
            # #########################################
            def irrelevant(id):
                # this functions replies true if the actor is too far away from the circle and is irrelevant
                loc = next((x for x in world.get_actors() if x.id == id), None).get_location()
                dist = np.linalg.norm([loc.x, loc.y])
                return dist > 40

            irrelevant_actors = [ac for ac in actor_list if irrelevant(ac)]
            delete_batch = []
            for ir_ac in irrelevant_actors:
                delete_batch.append(carla.command.DestroyActor(ir_ac))
                actor_list.remove(ir_ac)

            # #########################################
            # apply batches
            # #########################################
            count = 0
            for response in client.apply_batch_sync(spawn_batch):
                if response.error:
                    logging.error(response.error)
                else:
                    actor_list.append(response.actor_id)
                    count += 1

            print('spawned %d vehicles, press Ctrl+C to exit.' % count)

            # do the destroy batch here
            for response in client.apply_batch_sync(delete_batch):
                if response.error:
                    logging.error(response.error)
                # else:
                #     actor_list.remove()


            print('deleted irrelevant vehicles. ')


        while True:
            time.sleep(args.delay)
            try_update_actors()

        while True:
            world.wait_for_tick()

    finally:

        print('\ndestroying %d actors' % len(actor_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
