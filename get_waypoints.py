import carla

import math

class PossiblePaths(object):
    """
    PossiblePaths enables the user to calculate and display all available
     paths a Carla vehicle can take from its given location based on the
     legal driving maneuvers.
    """
    def __init__(self, vehicle, dist = 0.2, path_length = 10):
        self.m_vehicle = vehicle
        self.m_map = self.m_vehicle.get_world().get_map()
        self.m_dist = dist
        self.m_path_length = path_length
        self.m_paths = []

    def draw():
        draw_paths(self.m_paths)
    
    def compute_available_paths():
        self.m_paths = compute_all_paths(
            self.m_vehicle,
            self.m_map,
            self.m_dist,
            self.m_path_length
        )


def compute_all_paths(
    vehicle,
    map,
    dist_between_waypoints,
    path_length = 10,
    debug = False
):
    paths = []
    world = vehicle.get_world()
    map = world.get_map()
    path_start = map.get_waypoint(vehicle.get_location())
    add_to_path(path_start, paths, [], 0, dist_between_waypoints, path_length)

    if debug:
        draw_paths(paths, vehicle.get_world())

    return paths


def add_to_path(waypoint, paths, path, path_length, dist = 0.2, length = 10):
    if len(path) > path_length:
        path[path_length] = waypoint
    else:
        path.append(waypoint)

    path_length = path_length + 1

    if len(path) < length:
        for next in waypoint.next(dist):
            add_to_path(next, paths, path, path_length, dist, length)
    else:
        print(path)
        paths.append(path)


def draw_paths(paths, world):
    for x, path in enumerate(paths, start=1):
        draw_waypoints(world, path, 0.1*x)


def draw_waypoints(world, waypoints, z=0.5):
    """
    Draw a list of waypoints at a certain height given in z.

    :param world: carla.world object
    :param waypoints: list or iterable container with the waypoints to draw
    :param z: height in meters
    :return:
    """
    for w in waypoints:
        t = w.transform
        begin = t.location + carla.Location(z=z)
        angle = math.radians(t.rotation.yaw)
        end = begin + carla.Location(x=math.cos(angle), y=math.sin(angle))
        world.debug.draw_arrow(begin, end, arrow_size=0.3, life_time=1.0)