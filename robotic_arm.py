import math
import numpy as np
import scipy.optimize as opt


def generate_initial_vectors(n):
    """I assume that the robotic arm is anchored at the origin
    @return type: list"""
    return [np.array((0.0, 0.0))] + [np.random.rand(2) for i in range(1, n+1)]


def rotation_around_origin(v, angle):
    """Rotation of vector v by angle, around (0,0)
    @type  v: np.array or tuple
    @type  angle: float
    @return type: np.array"""
    rotation_matrix = np.array([[math.cos(angle), - math.sin(angle)],
                                [math.sin(angle), math.cos(angle)]])
    return rotation_matrix.dot(v)


def rotation_around_p(v, angle, p):
    """Rotation of vector v by angle, around point p
    @type  v: np.array or tuple
    @type  angle: float
    @type  p: np.array or tuple
    @return type: np.array"""
    return rotation_around_origin(v - p, angle) + p


def compute_position_of_joints_from_angles(vectors, angles):
    """ This is a main function.
    Computing a position of joints of robotic arm,
    by rotating it by proper angles.
    Last element in returned list is an end effector
    @type  vectors: list of np.arrays or tuples
    @type  angles: list of floats or array of floats
    @return type: np.array of np.arrays"""
    n = len(vectors)
    joints = np.cumsum(vectors, axis=0)  # to get co-ordinates of joint vectors
    for i, angle in enumerate(angles):
        for j in range(i+1, n):  # except origin, each joint is rotate at least by one angle around preceding joint
            joints[j] = rotation_around_p(joints[j], angle, joints[i])
    return joints


def distance_to_minimise(target_point, vectors, angles):
    """Distance between target_point and tip of n-robotic arm (end effector)
    after rotating by angles.
    Function is crucial 
    @type  target_point: np.array or tuple
    @type  vectors: list of np.arrays or tuples
    @type  angles: list of floats or array of floats
    @return type: float"""
    joints = compute_position_of_joints_from_angles(vectors, angles)
    return np.linalg.norm(joints[-1] - target_point)


def optimize_joints_angles(target_point, vectors, guess):
    """Finding best angles to rotate n-robotic arm
    to reach target point
    @type  target_point: np.array or tuple
    @type  vectors: list of np.arrays or tuples
    @type  guess: list of floats or array of floats
    @return type: np.array"""
    return opt.minimize(lambda alfa:
                        distance_to_minimise(target_point, vectors, alfa), guess).x


def is_target_point_reachable(target_point, vectors):
    """Cheking if our target is reachable for initial n-robotic arm
    @type  target_point: np.array or tuple
    @type  vectors: list of np.arrays or tuples
    @return type: bool"""
    distance_to_tagret = np.linalg.norm(np.array((0.0, 0.0)) - target_point)
    distances_combined = sum([np.linalg.norm(v) for v in vectors])
    return distance_to_tagret < distances_combined


if __name__ == "__main__":
    pass
