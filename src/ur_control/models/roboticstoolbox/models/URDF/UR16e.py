"""UR16e model generated from the installed official ROS ur_description package."""

import numpy as np
from roboticstoolbox.robot.Robot import Robot


class UR16e(Robot):
    """Nominal UR16e geometry, in the controller Base frame, ending at tool0."""

    def __init__(self):
        links, _, urdf_string, urdf_filepath = self.URDF_read(
            'axisforge_ur16e/ur16e.urdf'
        )
        super().__init__(
            links,
            name='UR16e',
            manufacturer='Universal Robots',
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )
        self.qr = np.array([np.pi, 0, 0, 0, np.pi / 2, 0])
        self.qz = np.zeros(6)
        self.addconfiguration('qr', self.qr)
        self.addconfiguration('qz', self.qz)
