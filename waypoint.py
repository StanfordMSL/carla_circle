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
from matplotlib.ticker import MultipleLocator
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
        waypoints = world.get_map().get_topology()
        dense_waypoint = map.generate_waypoints(20.0)
        # print("these are all the waypoints", dense_waypoint)
        dense_mat = []
        for waypt in dense_waypoint:
            dense_mat.append([waypt.transform.location.x, waypt.transform.location.y])
        dense_mat = np.asarray(dense_mat)
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.scatter(dense_mat[:,0], -dense_mat[:,1],s=1)
        # plt.show()
        #
        #
        #
        waypoints_Set = set()
        topo_waypt_mat = []
        #
        # fig = plt.figure()
        for first_pt, second_pt in waypoints:
            pt1 = (round(first_pt.transform.location.x, 2), round(first_pt.transform.location.y, 2))
            pt2 = (round(second_pt.transform.location.x, 2), round(second_pt.transform.location.y, 2))
            # if np.linalg.norm(pt1) < 60 and np.linalg.norm(pt2) < 60:
                # waypoints_Set.add(pt1)
                # waypoints_Set.add(pt2)
                # plt.text(first_pt.transform.location.x, first_pt.transform.location.y, s="l:" + str(first_pt.lane_id))
                # plt.text(first_pt.transform.location.x, first_pt.transform.location.y+2, s="r:" + str(first_pt.road_id))
                # print("here is the first point", first_pt.transform.location.x, first_pt.transform.location.y, " its road id", first_pt.road_id)
                # plt.text(first_pt.transform.location.x, first_pt.transform.location.y+2, s="lane width:" + str(first_pt.lane_width))
            if np.linalg.norm(pt1) < 80 and np.linalg.norm(pt2) < 80:
                topo_waypt_mat.append([pt1[0], pt1[1], pt2[0], pt2[1]])

                ax.plot([first_pt.transform.location.x, second_pt.transform.location.x], \
                    [-first_pt.transform.location.y, -second_pt.transform.location.y])





        entran1 = np.array([[39.49089813232422, -6.140676975250244], [37.86580276489258, -6.164945125579834], [36.23534393310547, -6.1922078132629395], [34.60498046875, -6.224769115447998], [32.9747314453125, -6.26262903213501], [31.344619750976562, -6.305786609649658], [29.71465301513672, -6.354242324829102], [28.08485221862793, -6.407994747161865], [26.376182556152344, -6.484718322753906], [24.543386459350586, -6.760074138641357], [22.76239585876465, -7.272933006286621], [21.063844680786133, -8.014472961425781], [19.476953506469727, -8.97193717956543], [18.029024124145508, -10.128852844238281], [16.74496078491211, -11.46532154083252], [15.698328971862793, -12.781710624694824], [14.727887153625488, -13.976324081420898], [13.697672843933105, -15.071495056152344], [12.584242820739746, -16.167631149291992], [11.456024169921875, -17.139192581176758], [10.2559175491333, -18.020414352416992], [8.991138458251953, -18.805997848510742], [7.669292449951172, -19.491222381591797], [6.2983269691467285, -20.071964263916016], [4.8864850997924805, -20.54473304748535], [3.442255735397339, -20.906688690185547], [1.974323034286499, -21.15565299987793], [0.49151328206062317, -21.290124893188477], [-0.9974474906921387, -21.309297561645508], [-2.4832282066345215, -21.21304702758789], [-4.073469638824463, -21.027658462524414], [-5.648009300231934, -20.836231231689453], [-7.1423821449279785, -20.579166412353516], [-8.617968559265137, -20.230056762695312], [-10.069106101989746, -19.790241241455078], [-11.49022388458252, -19.261417388916016], [-12.875866889953613, -18.64560317993164], [-14.220718383789062, -17.945167541503906], [-15.551539421081543, -17.150056838989258], [-16.907651901245117, -16.298736572265625]])
        entran2 = np.array([[-4.817941188812256, -39.5645751953125], [-4.931615829467773, -37.914188385009766], [-5.072919845581055, -36.26593780517578], [-5.2418131828308105, -34.62028884887695], [-5.438249588012695, -32.977691650390625], [-5.662173271179199, -31.338623046875], [-5.913521766662598, -29.703533172607422], [-6.2060627937316895, -28.0336856842041], [-6.633067607879639, -26.313678741455078], [-7.203592300415039, -24.63580322265625], [-7.913572788238525, -23.01201629638672], [-8.757951736450195, -21.453880310058594], [-9.730714797973633, -19.972497940063477], [-10.824929237365723, -18.578420639038086], [-12.032805442810059, -17.281579971313477], [-13.345735549926758, -16.091211318969727], [-14.59863567352295, -15.06821060180664], [-15.700803756713867, -14.067193984985352], [-16.814422607421875, -12.950329780578613], [-17.805749893188477, -11.83944034576416], [-18.708024978637695, -10.65507984161377], [-19.515817642211914, -9.40436840057373], [-20.224273681640625, -8.094826698303223], [-20.829132080078125, -6.734328746795654], [-21.32675552368164, -5.331054210662842], [-21.714153289794922, -3.8934409618377686], [-21.988996505737305, -2.430133104324341], [-22.149629592895508, -0.9499288201332092], [-22.19508934020996, 0.5384615063667297], [-22.125085830688477, 2.025710344314575], [-21.940059661865234, 3.503063440322876], [-21.641117095947266, 4.961638927459717], [-21.230058670043945, 6.392666339874268], [-20.7093563079834, 7.787541389465332], [-20.082138061523438, 9.13787841796875], [-19.3521785736084, 10.435554504394531], [-18.52386474609375, 11.672771453857422], [-17.602176666259766, 12.84208869934082], [-16.59265899658203, 13.936474800109863], [-15.400768280029297, 15.188582420349121]])
        entran3 = np.array([[-34.814674377441406, -0.47130560874938965], [-33.29544448852539, -0.15828955173492432], [-31.790752410888672, 0.21845054626464844], [-30.303266525268555, 0.6582458019256592], [-28.937562942504883, 1.1595783233642578], [-27.651111602783203, 1.7960319519042969], [-26.347867965698242, 2.602447032928467], [-25.255699157714844, 3.4120240211486816], [-24.312442779541016, 4.375119209289551], [-23.48154067993164, 5.530763626098633], [-22.652679443359375, 6.847446441650391], [-21.87640380859375, 8.195799827575684], [-21.102096557617188, 9.63101863861084], [-20.19000244140625, 11.136578559875488], [-19.16399574279785, 12.566939353942871], [-18.030244827270508, 13.913501739501953], [-16.862815856933594, 15.119731903076172], [-15.867419242858887, 16.229799270629883], [-14.959613800048828, 17.41257667541504], [-14.14469051361084, 18.661163330078125], [-13.427399635314941, 19.96828269958496], [-12.81192398071289, 21.326316833496094], [-12.301850318908691, 22.72734832763672], [-11.900151252746582, 24.1632137298584], [-11.59899616241455, 25.65958595275879], [-11.310633659362793, 27.2587947845459], [-11.031770706176758, 28.83707618713379], [-10.788163185119629, 30.403366088867188], [-10.580278396606445, 31.974796295166016], [-10.408222198486328, 33.55055236816406], [-10.272087097167969, 35.12981414794922], [-10.171940803527832, 36.711769104003906], [-10.107836723327637, 38.29559326171875], [-10.079802513122559, 39.880516052246094], [-10.069519996643066, 41.5054817199707], [-10.05923843383789, 43.13045120239258], [-10.048955917358398, 44.75541687011719], [-10.038674354553223, 46.38038635253906], [-10.02839183807373, 48.00535202026367], [-10.018110275268555, 49.63032150268555]])
        entran4 = np.array([[3.2099719047546387, 39.47578811645508], [3.321388006210327, 37.78154754638672], [3.503384828567505, 36.09342575073242], [3.755645751953125, 34.414371490478516], [4.077731609344482, 32.74729537963867], [4.469081878662109, 31.095111846923828], [4.925609588623047, 29.469890594482422], [5.389627456665039, 27.912548065185547], [5.935444355010986, 26.26019859313965], [6.607458114624023, 24.638778686523438], [7.398468971252441, 23.071962356567383], [8.304075241088867, 21.56847381591797], [9.319238662719727, 20.136672973632812], [10.438308715820312, 18.784530639648438], [11.655058860778809, 17.519569396972656], [12.962719917297363, 16.34882926940918], [14.146451950073242, 15.350162506103516], [15.250914573669434, 14.278253555297852], [16.309186935424805, 13.17756462097168], [17.25775146484375, 12.029940605163574], [18.114538192749023, 10.812270164489746], [19.139684677124023, 9.346755981445312], [20.363069534301758, 7.976857662200928], [21.743579864501953, 6.765460014343262], [23.26082992553711, 5.730450630187988], [24.89241600036621, 4.887112617492676], [26.61424446105957, 4.247899532318115], [28.400896072387695, 3.822249412536621], [30.225982666015625, 3.616448163986206], [31.93506622314453, 3.5933148860931396], [33.56005859375, 3.588041067123413], [35.18888854980469, 3.5839576721191406], [36.819576263427734, 3.5848898887634277], [38.45024871826172, 3.5911219120025635], [40.08089828491211, 3.6026535034179688], [41.71149444580078, 3.6194849014282227], [43.34157943725586, 3.641594171524048], [44.966392517089844, 3.665865182876587], [46.591209411621094, 3.690136194229126], [48.216026306152344, 3.714406967163086]])

        ax.plot(entran1[:,0], -entran1[:,1], 'r', linewidth=5)
        ax.plot(entran2[:,0], -entran2[:,1], 'b', linewidth=5)
        ax.plot(entran3[:,0], -entran3[:,1], 'g', linewidth=5)
        ax.plot(entran4[:,0], -entran4[:,1], 'y', linewidth=5)
        # print("this is the whole list", map)
        # print("these are all the waypoints", len(waypoints_Set))
        # print("----------------")
        # print(np.asarray(list(waypoints_Set)))
        # waypoint_tree = KDTree(np.asarray(list(waypoints_Set)))
        spacing = 10
        minorLocator = MultipleLocator(spacing)
        ax.xaxis.set_minor_locator(minorLocator)
        ax.yaxis.set_minor_locator(minorLocator)
        ax.grid(which="minor")


        # -----------------------------------
        # print all the spawn points
        # -----------------------------------

        # spawn_points = map.get_spawn_points()
        # print("all the points", spawn_points)
        # spawn_points_mat = []
        # for s_point in spawn_points:
        #     spawn_points_mat.append([s_point.location.x, s_point.location.y])
        # spawn_points_mat = np.asarray(spawn_points_mat)
        # plt.scatter(spawn_points_mat[:,0], -spawn_points_mat[:,1])
        # plt.show()



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

        # waypoints = map.generate_waypoints(5.0)
        # fig = plt.figure()
        # ax = fig.add_subplot(111)
        # waypoints_list = []
        # ang = []
        # for wy in waypoints:
        #     dist = np.linalg.norm([wy.transform.location.x, wy.transform.location.y])
        #     if dist < 50:
        #         plt.scatter(wy.transform.location.x, -wy.transform.location.y)
        #         waypoints_list.append([wy.transform.location.x, -wy.transform.location.y])
        #         ang.append(np.arctan2(wy.transform.location.y, wy.transform.location.x))
        # waypoints_array = np.asarray(waypoints_list)
        # sort_index = np.argsort(ang)
        # sorted_waypoints = waypoints_array[sort_index, :]
        # print("these are the sorted waypoits", sorted_waypoints)
        # # plt.plot(sorted_waypoints[:,0], sorted_waypoints[:,1])
        #
        #
        # radius = 20
        # ang_increment = 2* np.pi / 300
        # ang = [i * ang_increment for i in range(300)]
        # circle = np.zeros((300, 2))
        # circle[:,0] = radius * np.cos(ang) - 0.5
        # circle[:,1] = radius * np.sin(ang) - 0.3
        # plt.plot(circle[:,0], circle[:,1])
        # ticks = np.arange(-40, 40, 5)
        # ax.set_xticks(ticks)
        # ax.set_yticks(ticks)
        # plt.grid()
        # plt.axis("equal")
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
