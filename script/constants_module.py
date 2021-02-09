"""This module defines several constants used throughout the AV stack.

All values are in SI units.

Parameters
----------
MAX_ACCELERATION : float
    The maximum comfortable acceleration the vehicle should experience in
    normal driving situations. 3.0 [:math:`\\frac{m}{s^2}`]
MAX_DECELERATION : 9.0 [:math:`\\frac{m}{s^2}`]
    The maximum deceleration the vehicle should experience when in emergency
    situations.
MAX_JERK : 1.3 [:math:`\\frac{m}{s^3}`]
    The maximum jerk the vehicle should experience in normal driving
    situations.
FREEFLOW_SPEED : 1.0 [:math:`\\frac{1}{s}`]
    The proportional constant when in freeflow driving scenarios (car
    following).
MODE_NORMAL : 0
    Mode representing the vehicle when in normal driving situations.
MODE_EMERGENCY : 1
    Mode representing the vehicle when in emergency driving situations.
"""
MAX_ACCELERATION = 3.0  # [m/s^2] (positive)
MAX_DECELERATION = 9.0  # [m/s^2] (positive)
MAX_JERK = 1.3  # [m/s^3] (positive)
FREEFLOW_SPEED = 1.0  # [1/s] (proportional constant when in freeflow)

# Once using python 3.4+ this should be an enum
MODE_NORMAL = 0
MODE_EMERGENCY = 1
