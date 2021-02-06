# Module to define several constants used throughout the AV stack
MAX_ACCELERATION = 3.0  # [m/s^2] (positive)
MAX_DECELERATION = 9.0  # [m/s^2] (positive)
MAX_JERK = 1.3  # [m/s^3] (positive)
FREEFLOW_SPEED = 1.0  # [1/s] (proportional constant when in freeflow)

# Once using python 3.4+ this should be an enum
MODE_NORMAL = 0
MODE_EMERGENCY = 1
