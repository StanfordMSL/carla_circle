'''
Utility functions related to a Carla World object.
'''
import carla
import numpy
import time


def visualize_location_list(
    world: carla.World,
    location_list,
    timeout: float = 10.0,
    pause: bool = False
) -> None:
    '''
    Draws points of each location contained in the list into the Carla World.

    Parameters
    ----------
    world : carla.World
        The Carla World in which to draw the points.
    location_list : numpy.ndarray of shape(# of points, 2)
        The list of locations to visualize, stored in a numpy ndarray with all
        the points stored as two floats representing the 2D location in space.
    timeout : float, optional
        The time in which the points should stay visible, by default 10.0
    pause: bool, optional
        Determines whether the visualization should pause briefly after each
        point is drawn.
    '''
    for location in location_list:
        world.debug.draw_point(
            carla.Location(location[0], location[1], 0.0),
            life_time=timeout,
            size=0.05
        )

        if pause:
            time.sleep(0.1)
