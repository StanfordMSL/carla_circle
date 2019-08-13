import utilities.world

import carla
import math
import numpy

carla_world = None

def generate_circle_waypoints(number_of_points, debug=False):
    waypoints = numpy.empty([number_of_points, 2])
    circle_radius = 21.5

    for i in range(number_of_points):
        a = -2.0*i*math.pi/number_of_points
        x = circle_radius*math.cos(a)
        y = circle_radius*math.sin(a)

        wp = [x, y]
        waypoints[i, :] = wp

    if debug:
        print("Number of circle waypoints:", len(waypoints))
        print(waypoints)

    return waypoints


def get_next_waypoint(waypoint, step_size):
    next_waypoint = waypoint.next(step_size)[0]

    return next_waypoint


def generate_entry_waypoints(
    initial_location: carla.Location,
    offset: carla.Vector3D = carla.Vector3D(),
    number_of_points: int = 10,
    distance: float = 40.0,
    debug: bool = False
):
    waypoints = numpy.empty([number_of_points, 2])
    step_size = distance/number_of_points
    waypoint = carla_world.get_map().get_waypoint(initial_location)

    for i in range(number_of_points):
        carla_location = waypoint.transform.location
        center_location = carla.Location(
            carla_location.x + offset.x,
            carla_location.y + offset.y
        )
        waypoints[i, :] = [center_location.x, center_location.y]
        waypoint = get_next_waypoint(waypoint, step_size)

    if debug:
        print("Number of entering waypoints:", len(waypoints))
        print(waypoints)

    return waypoints


def main():
    global carla_world

    carla_client = carla.Client('localhost', 2000)
    carla_world = carla_client.get_world()
    carla_map = carla_world.get_map()

    weather = carla.WeatherParameters(
        cloudyness=0.0,
        precipitation=0.0,
        precipitation_deposits=0.0,
        wind_intensity=0.0,
        sun_azimuth_angle=130.0,
        sun_altitude_angle=68.0)
    carla_world.set_weather(weather)

    debug_flag = False
    delay_flag = False
    draw_flag = False

    # Generate waypoints we care about for Event 4
    circle_waypoints = generate_circle_waypoints(100, debug_flag)

    entry_location_1 = carla.Location(39.5, -5.0, 0.0)
    entry_waypoints_1 = generate_entry_waypoints(
        entry_location_1,
        offset=carla.Vector3D(0.0, -1.75, 0.0),
        number_of_points=20,
        distance=20.0,
        debug=debug_flag
    )

    entry_location_2 = carla.Location(-2.0, -39.5, 0.0)
    entry_waypoints_2 = generate_entry_waypoints(
        entry_location_2,
        offset=carla.Vector3D(-1.75, 0.0, 0.0),
        number_of_points=20,
        distance=20.0,
        debug=debug_flag
    )

    entry_location_3 = carla.Location(-35.5, 5.0, 0.0)
    entry_waypoints_3 = generate_entry_waypoints(
        entry_location_3,
        offset=carla.Vector3D(0.0, -1.75, 0.0),
        number_of_points=20,
        distance=30.0,
        debug=debug_flag
    )

    entry_location_4 = carla.Location(2.0, 39.5, 0.0)
    entry_waypoints_4 = generate_entry_waypoints(
        entry_location_4,
        offset=carla.Vector3D(1.75, 0.0, 0.0),
        number_of_points=20,
        distance=20.0,
        debug=debug_flag
    )

    exit_location = carla.Location(18.5, 8.5, 0.0)
    exit_waypoints = generate_entry_waypoints(
        exit_location,
        offset=carla.Vector3D(0.0, 1.75, 0.0),
        number_of_points=20,
        distance=20.0,
        debug=debug_flag
    )

    # Write waypoints to their respective files
    circle_waypoints.tofile("resource/cirle_waypoints.nparray")
    entry_waypoints_1.tofile("resource/entry_waypoints_1.nparray")
    entry_waypoints_2.tofile("resource/entry_waypoints_2.nparray")
    entry_waypoints_3.tofile("resource/entry_waypoints_3.nparray")
    entry_waypoints_4.tofile("resource/entry_waypoints_4.nparray")

    # Visualize the generated waypoints
    if draw_flag:
        utilities.world.visualize_location_list(
            carla_world,
            circle_waypoints,
            pause=delay_flag
        )
        utilities.world.visualize_location_list(
            carla_world,
            entry_waypoints_1,
            pause=delay_flag
        )
        utilities.world.visualize_location_list(
            carla_world,
            entry_waypoints_2,
            pause=delay_flag
        )
        utilities.world.visualize_location_list(
            carla_world,
            entry_waypoints_3,
            pause=delay_flag
        )
        utilities.world.visualize_location_list(
            carla_world,
            entry_waypoints_4,
            pause=delay_flag
        )
        utilities.world.visualize_location_list(
            carla_world,
            exit_waypoints,
            pause=delay_flag
        )
    

if __name__ == '__main__':
    main()
