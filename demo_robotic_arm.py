import math
import matplotlib.pyplot as plt
import numpy as np
import robotic_arm

T = np.linspace(0, 2*math.pi, num=100, endpoint=True)  # We set up lenght of T
axis_runcloid = [-1.5, 1.5, -1.5, 1.5]
axis_circle = [-3.0, 3.0, -3.0, 3.0]
axis_spiral = [-3.0, 3, -3.0, 3]


def parametric_circle_point(t):
    """Parametric representation of the circle of radius r,
    where t is the parameter (angle).
    It returns one point on parametric curve.
    @type  t: float
    @return type: np.arrays"""
    r = 2
    return np.array((r*math.cos(t), r*math.sin(t)))


def parametric_spiral_point(t):
    """Parametric representation of the spiral,
    where t is the parameter (angle) and K is constant
    It returns one point on parametric curve.
    @type  t: float
    @return type: np.arrays"""
    K = 0.3
    return np.array((K*t*math.cos(t), K*t*math.sin(t)))


def parametric_ranunculoid_point(t):
    """Parametric representation of the ranunculoid of radius r,
    where t is the parameter (angle).
    Ranunculoid looks like flower.
    It returns one point on parametric curve.
    @type  t: float
    @return type: np.arrays"""
    r = 1
    return np.array(((-6.0/5)*r*math.cos(t) + (1.0/5)*r*math.cos(6*t),
                     (-6.0/5)*r*math.sin(t) + (1.0/5)*r*math.sin(6*t)))


def demo_n_robotic_arm(vectors, parametric_p, T, axis, save_img = True):
    """Findind the optimal position of n-robotic arm (n-joint vectors)
    for reaching the target points on a given curve,
    we set up axis limit [x1, x2, y1, y2] for plot
    @type  vectors: list of np.arrays or tuples
    @type  parametric_p: function
    @type  T: np.array of floats, we set up lenght of T
    @type  axis: list
    @return type: plot"""
    angles = [math.pi/4]*(len(vectors)-1)  # Initial guess for optimize function
    for i, t in enumerate(T):
        point_on_curve = parametric_p(t)
        plt.scatter(point_on_curve[0], point_on_curve[1], color="r")
        if not robotic_arm.is_target_point_reachable(point_on_curve, vectors):
            raise ValueError("Robotic Arm couldn't reach the target")
        angles = robotic_arm.optimize_joints_angles(point_on_curve, vectors, angles)  # updating initial guess
        joints = robotic_arm.compute_position_of_joints_from_angles(vectors, angles)
        plt.plot(*zip(*joints), color="b")
        if save_img:
            plt.axis(axis)
            for j in np.linspace(0, 2*math.pi, num=100, endpoint=True):
                plt.scatter(parametric_p(j)[0], parametric_p(j)[1], color="r")
            plt.savefig(r"{0:0>5}.png".format(i))
            plt.clf()


def demo_compare_robots(n, m, parametric_curve, T, angle, axis):
    """To compare m different robotic arms reach for the same target point
    (just for one input angle)
    n - numbers of initial join vectors , m - numbers of robots
    @type  n: int
    @type  m: int
    @type  parametric_curve: function
    @type  T: np.array of floats, we set up lenght of T
    @type angle: float
    @type  axis: list
    @return type: plot"""
    plt_size = int(math.ceil(math.sqrt(m)))
    for i in range(m):
        v = robotic_arm.generate_initial_vectors(n)
        plt.subplot(plt_size, plt_size, i+1)
        demo_n_robotic_arm(v, parametric_curve, [angle], axis)
        for t in T:
            plt.scatter(parametric_curve(t)[0], parametric_curve(t)[1], color="r")
    # plt.savefig(r"C:\Users\gosia\Desktop\python\algebraliniowa\{}.png".format(angle))


if __name__ == "__main__":
    vectors = robotic_arm.generate_initial_vectors(4)
    demo_n_robotic_arm(vectors, parametric_ranunculoid_point, T, axis_runcloid, save_img = True)
    plt.show()
    
    
    

