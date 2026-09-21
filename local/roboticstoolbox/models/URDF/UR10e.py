#!/usr/bin/env python

import numpy as np
from roboticstoolbox.robot.Robot import Robot
from math import pi


class UR10e(Robot):
    """
    Class that imports a UR10e URDF model

    ``UR3()`` is a class which imports a Universal Robotics UR5 robot
    definition from a URDF file.  The model describes its kinematic and
    graphical characteristics.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.URDF.UR10e()
        >>> print(robot)

    Defined joint configurations are:

    - qz, zero joint angle configuration, 'L' shaped configuration
    - qr, vertical 'READY' configuration

    .. codeauthor:: Jesse Haviland
    .. sectionauthor:: Peter Corke
    """

    def __init__(self):

        links, name, urdf_string, urdf_filepath = self.URDF_read(
            "ur_description/urdf/ur10e_joint_limited_robot.urdf.xacro"
        )
        # for link in links:
        #     print(link)

        super().__init__(
            links,
            name="UR10e",
            manufacturer="Universal Robotics",
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )

        self.qr = np.array([np.pi, 0, 0, 0, np.pi / 2, 0])
        self.qz = np.zeros(6)

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)

if __name__ == "__main__":  # pragma nocover

    robot = UR10e()
    print(robot)
    #print(robot.ets())
