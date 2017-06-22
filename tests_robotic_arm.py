import matplotlib.pyplot as plt
import numpy as np
import math
import scipy.optimize as opt
import robotic_arm


def tests():

    test_rotation_around_origin()
    test_rotation_around_p()
    test_compute_position_of_joints_from_angles()
    test_distance_to_minimise()
    test_optimize_joints_angles()
    test_is_target_point_reachable_true()
    test_is_target_point_reachable_false()


def test_rotation_around_origin():
    v = np.array((10, 10))
    angle = math.pi
    Eps = 1e-6
    assert abs(robotic_arm.rotation_around_origin(v, angle)[0] - (-10)) < Eps
    assert abs(robotic_arm.rotation_around_origin(v, angle)[1] - (-10)) < Eps


def test_rotation_around_p():
    v = np.array((10, 10))
    angle = math.pi
    p = (1, 1)
    Eps = 1e-6
    assert abs(robotic_arm.rotation_around_p(v, angle, p)[0] - (-8)) < Eps
    assert abs(robotic_arm.rotation_around_p(v, angle, p)[1] - (-8)) < Eps


def test_compute_position_of_joints_from_angles():
    vectors = [np.array((0.0, 0.0)), np.array((1.0, 0.0)),
               np.array((2.0, 0.0))]
    angles = [math.pi/2, math.pi/2]
    Eps = 1e-6
    joints = robotic_arm.compute_position_of_joints_from_angles(vectors, angles)
    assert np.linalg.norm(joints[0] - np.array((0.0, 0.0))) < Eps
    assert np.linalg.norm(joints[1] - np.array((0.0, 1.0))) < Eps
    assert np.linalg.norm(joints[2] - np.array((-2.0, 1.0))) < Eps


def test_distance_to_minimise():
    vectors = [np.array((0.0, 0.0)), np.array((1.0, 0.0)),
               np.array((2.0, 0.0)), np.array((3.0, 0.0))]
    angles = [math.pi/2, math.pi/2, math.pi/2]
    Eps = 1e-6
    assert robotic_arm.distance_to_minimise((-2.0, 0.0), vectors, angles) - 2 < Eps


def test_optimize_joints_angles():
    vectors = [np.array((0.0, 0.0)), np.array((1.0, 0.0)),
               np.array((2.0, 0.0)), np.array((3.0, 0.0))]
    target_point = (-2.0, 0.0)
    guess = [math.pi/4] * 3
    angles = robotic_arm.optimize_joints_angles(target_point, vectors, guess)
    Eps = 1e-6
    assert abs(angles[0] - 3.10689704) < Eps
    assert abs(angles[1] - 3.2709563) < Eps
    assert abs(angles[2] - 3.12157455) < Eps


def test_is_target_point_reachable_true():
    vectors = [np.array((0.0, 0.0)), np.array((1.0, 0.0)),
               np.array((2.0, 0.0)), np.array((3.0, 0.0))]
    target_point = (2.0, 2.0)
    assert robotic_arm.is_target_point_reachable(target_point, vectors) == True


def test_is_target_point_reachable_false():
    vectors = [np.array((0.0, 0.0)), np.array((1.0, 0.0)),
               np.array((2.0, 0.0)), np.array((3.0, 0.0))]
    target_point = (200.0, 200.0)
    assert robotic_arm.is_target_point_reachable(target_point, vectors) == False


if __name__ == "__main__":
    tests()
