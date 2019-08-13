import carla

from enum import Enum
import math
import matplotlib.pyplot as plt

class PossiblePaths(object):
    '''
    PossiblePaths enables the user to calculate and display all available
    paths a Carla vehicle can take from its given location based on the
    legal driving maneuvers.
    
    Parameters
    ----------
    object : [type]
        [description]
    '''
    def __init__(
        self,
        vehicle: carla.Actor,
        dist:float = 0.2,
        path_length: int = 10
    ):
        self.m_vehicle = vehicle
        self.m_map = self.m_vehicle.get_world().get_map()
        self.m_dist = dist
        self.m_path_length = path_length
        self.m_paths = []

    def draw(self):
        draw_paths(self.m_paths)
    
    def compute_available_paths(self):
        self.m_paths = compute_all_paths_for_actor(
            self.m_vehicle,
            self.m_map,
            self.m_dist,
            self.m_path_length
        )


def compute_all_paths_from_location(
    location: carla.Location,
    carla_world: carla.World,
    dist_between_waypoints: float,
    path_length: int = 10,
    verbose: bool = False
):
    paths = []
    path_start = carla_world.get_map().get_waypoint(location)
    
    if verbose:
        print("Path start - ")
        print_waypoint_info(path_start)

    add_to_path(
        path_start,
        paths,
        [],
        0,
        dist_between_waypoints,
        path_length,
        verbose
    )
    
    if verbose:
        draw_paths(paths, carla_world)

    return paths


def compute_all_paths_for_actor(
    vehicle: carla.Actor,
    dist_between_waypoints: float,
    path_length: int = 10,
    verbose: bool = False
):
    world = vehicle.get_world()
    location = vehicle.get_location()
    paths = compute_all_paths_from_location(
        vehicle.get_location,
        world,
        dist_between_waypoints,
        path_length,
        verbose
    )

    return paths


def add_to_path(
    waypoint: carla.Waypoint,
    paths: type([]),
    path: type([]) = [],
    path_length: float = 0,
    dist: float = 0.2,
    length: int = 10,
    verbose: bool = False
):
    if len(path) > path_length:
        path[path_length] = waypoint
    else:
        path.append(waypoint)

        if verbose:
            print_waypoint_info(waypoint)
            print("Path so far:", path)

    path_length = path_length + 1

    if len(path) < length:
        next_waypoints = waypoint.next(dist)
        for i, wp in enumerate(next_waypoints):
            add_to_path(wp, paths, path, path_length, dist, length, verbose)
    else:
        paths.append(path)
        
        if verbose:
            print(path)


def draw_paths(paths, world):
    colors = [x.value for x in Colors]

    for x, path in enumerate(paths, start=1):
        draw_waypoints(world, path, 0.1*x, colors[x-1])


def draw_waypoints(
    world,
    waypoints,
    z=0.5,
    color=carla.Color(255,0,0),
    timeout=5.0
):
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
        world.debug.draw_arrow(
            begin,
            end,
            color=color,
            life_time=timeout,
            arrow_size=0.3
        )


def draw_waypoint(
    world,
    waypoint,
    z=0.5,
    color=carla.Color(255, 255, 255),
    timeout=5.0
):
    location = waypoint.transform.location + carla.Location(z=z)
    world.draw_point(location, color=color, life_time=timeout)


def print_waypoint_info(waypoint: carla.Waypoint):
    print(
        "Waypoint:\n"
        "   ID: {}\n"
        "   Transform: {} {}\n"
        "   Junction?: {}\n"
        "   Lane Width: {}m\n"
        "   Road ID: {}\n"
        "   Section ID: {}\n"
        "   Lane ID: {}\n"
        "   S: {}\n".format(
            waypoint.id,
            waypoint.transform.location, waypoint.transform.rotation,
            waypoint.is_junction,
            waypoint.lane_width,
            waypoint.road_id,
            waypoint.section_id,
            waypoint.lane_id,
            waypoint.s
        )
    )


def print_waypoint_location(waypoint: carla.Waypoint):
    print("Waypoint - {}".format(waypoint.transform.location))


class Colors(Enum):
    BLUE = carla.Color(0, 0, 255)
    RED = carla.Color(255, 0, 0)
    GREEN = carla.Color(0, 255, 0)
    CYAN = carla.Color(0, 255, 255)
    MAGENTA = carla.Color(255, 0, 255)
    YELLOW = carla.Color(255, 255, 0)