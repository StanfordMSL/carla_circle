import rospy
from nav_msgs.msg import Path
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Quaternion, Transform, Twist, Vector3

from constants_module import MAX_ACCELERATION

import numpy as np
import cvxpy as cvx


def vehicle_dynamics(dt):
    '''Defines the vehicle dynamics for use as a constraint in our
    optimization.

    We are currently using a single integrator system to represent the vehicle
    dynamics. :math:`\vec{x_{k+1}} = A*\vec{x_k} + B*\vec{u_k}`, where 
    :math:`\vec{x_{k}}^T = [x_k, y_k, v_{x_k}, v_{y_k}]` and
    :math:`\vec{u_{k}}^T = [a_{x_k}, a_{y_k}]`

    Parameters
    ----------
    dt : float
        The change in time over two consecutive steps.

    Returns
    -------
    A : numpy.array
        The state/system matrix representing part of the vehicle dynamics.
    B : numpy.array
        The input matrix representing part of the vehicle dynamics.
    '''
    A = np.array(
        [
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ]
    )
    B = np.array(
        [
            [0, 0],
            [0, 0],
            [dt, 0],
            [0, dt]
        ]
    )

    return A, B


def trajectory_optimization(trajectory_in, time_step):
    '''Runs an optimization on the given trajectory.

    Parameters
    ----------
    trajectory_in : MultiDOFJointTrajectory
        The input trajectory to optimize.
    time_step : float
        The time between steps or delta time.

    Returns
    -------
    MultiDOFJointTrajectory
        The optimal trajectory
    '''
    # Convert input trajectory to usable input for optimizer
    T = len(trajectory_in.points)
    x_in = np.ndarray((4, T))

    for i in range(T):
        x_in[0, i] = trajectory_in.points[i].transforms[0].translation.x
        x_in[1, i] = trajectory_in.points[i].transforms[0].translation.y
        x_in[2, i] = trajectory_in.points[i].velocities[0].linear.x
        x_in[3, i] = trajectory_in.points[i].velocities[0].linear.y

    A, B = vehicle_dynamics(time_step)
    x = cvx.Variable((4, T))
    u = cvx.Variable((2, T))

    cost = 0
    g = .0002
    constr = []
    for t in range(T - 1):
        cost += cvx.sum_squares(x_in[:2, t + 1] - x[:2, t + 1])
        cost += g*cvx.sum_squares(u[:, t+1])
        constr += [
            x[:, t + 1] == A * x[:, t] + B * u[:, t],
            # cvx.norm(u[:, t], 2) <= MAX_ACCELERATION
        ]
    # Sum problem objectives and concatenates constraints
    constr += [x[:2, 0] == x_in[:2, 0]]
    problem = cvx.Problem(cvx.Minimize(cost), constr)
    problem.solve(solver=cvx.ECOS)

    # Initialize our optimized trajectory message
    trajectory = MultiDOFJointTrajectory()
    trajectory.header.stamp = rospy.Time.now()
    trajectory.header.frame_id = 'map'
    trajectory.points = []

    for i in range(T):
        trajectory_point = MultiDOFJointTrajectoryPoint()
        trajectory_point.transforms = [
            Transform(
                Vector3(x[0, i].value, x[1, i].value, 0.0),
                Quaternion(0.0, 0.0, 0.0, 1.0)
            )
        ]
        trajectory_point.velocities = [
            Twist(
                Vector3(x[2, i].value, x[3, i].value, 0.0),
                Vector3(0.0, 0.0, 0.0)
            )
        ]
        trajectory_point.accelerations = [
            Twist(
                Vector3(u[0, i].value, u[1, i].value, 0.0),
                Vector3(0.0, 0.0, 0.0)
            )
        ]
        trajectory.points.append(trajectory_point)

    return trajectory
