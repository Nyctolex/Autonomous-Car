import numpy as np
from numpy.linalg import norm
from math import atan2

def hat(v :np.ndarray) -> np.ndarray:
    """ return the normilized vector of v """
    return v/norm(v)


def is_clock_wise_angle(v1: np.ndarray, v2: np.ndarray) -> bool:
    """ returns if v1 is clockwise to v2"""
    v1 = hat(v1)
    v2 = hat(v2)
    dot = v1[0]*-1*v2[1] + v1[1]*v2[0]
    return dot > 0

def non_negative_angle(angle: float) -> float:
    """Transfer a negative angle to it's positive version (in radians)"""
    angle = angle % 2*np.pi
    if angle > 0:
        return angle
    return 2*np.pi + angle

def inner_angle(v1:np.ndarray[np.float64, np.ndim == 2], v2: np.ndarray = np.array([1,0])):
    """ Returns the inner angle between v1 and v2. By default, it would be the angle with the x axis"""
    angle1 :float= non_negative_angle(atan2(v1[1], v1[0]))
    if v2 is np.array([1,0]):
        return angle1
    angle2:float = non_negative_angle(atan2(v2[1], v2[0]))
    angle:float = max(angle1, angle2) - min(angle1, angle2)
    if angle < np.pi:
        return angle
    return 2*np.pi - angle

def next_point_controller(point: np.ndarray[np.float64], position:np.ndarray, velocity_vector:np.ndarray) -> float:
    """Path controller which return the next direction for the car"""
    pointing_vector = point-position
    angle = inner_angle(velocity_vector, pointing_vector)
    if is_clock_wise_angle(pointing_vector,  velocity_vector):
        return angle
    return -1*angle
