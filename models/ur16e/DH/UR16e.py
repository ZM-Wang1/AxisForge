import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3


class UR16e(DHRobot):
    """
    Class that models a Universal Robots UR16e manipulator

    :param symbolic: use symbolic constants
    :type symbolic: bool

    ``UR16e()`` is an object which models a Universal Robots UR16e robot
    and describes its kinematic and dynamic characteristics using standard
    DH conventions.

    :References:
        - Parameters for calculations of kinematics and dynamics,
          https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
    """

    def __init__(self, symbolic=False):

        if symbolic:
            import spatialmath.base.symbolic as sym
            zero = sym.zero()
            pi = sym.pi()
        else:
            from math import pi
            zero = 0.0

        deg = pi / 180

        # robot length values (metres) - official UR16e DH parameters
        a = [0, -0.4784, -0.36, 0, 0, 0]
        d = [0.1807, 0, 0, 0.17415, 0.11985, 0.11655]

        alpha = [pi / 2, zero, zero, pi / 2, -pi / 2, zero]

        # mass data, no inertia available
        mass = [7.369, 10.450, 4.321, 2.180, 2.033, 0.907]
        center_of_mass = [
            [0.000, -0.016, 0.030],
            [0.302, 0.000, 0.160],
            [0.194, 0.000, 0.065],
            [0.000, -0.009, 0.011],
            [0.000, 0.018, 0.012],
            [0, 0, -0.044],
        ]
        links = []

        for j in range(6):
            link = RevoluteDH(
                d=d[j], a=a[j], alpha=alpha[j], m=mass[j], r=center_of_mass[j], G=1
            )
            links.append(link)

        super().__init__(
            links,
            name="UR16e",
            manufacturer="Universal Robots",
            keywords=("dynamics", "symbolic"),
            symbolic=symbolic,
        )

        self.qr = np.array([180, 0, 0, 0, 90, 0]) * deg
        self.qz = np.zeros(6)

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)


if __name__ == "__main__":
    robot = UR16e(symbolic=False)
    print(robot)
