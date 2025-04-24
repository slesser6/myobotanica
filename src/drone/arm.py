import roboticstoolbox as rtb
from spatialmath import *
import numpy as np
import logging
from src.utils import get_logger

class Arm(rtb.DHRobot):

    def __init__(self, cfg):
        self._logger = get_logger("Arm")
        if cfg.verbose:
            self._logger.setLevel(logging.DEBUG)
        else:
            self._logger.setLevel(logging.INFO)
        self._enable = cfg.enable

        L = [
            rtb.RevoluteDH(
                d = -2,
                alpha = np.pi/2,
                # qlim = [-np.pi, np.pi]
            ),
            rtb.RevoluteDH(
                a = 3,
                qlim = [-np.pi/2, 0]
            ),
            rtb.RevoluteDH(
                a = 3,
            )
        ]

        super().__init__(
            L,
            name="Arm",
            manufacturer="Myobotanica"
        )

        self._qn = np.array([0, -np.pi/4, np.pi/4]) #nominal/starting pose
        self._qz = np.array([0, 0, 0])

        self.addconfiguration("qn", self._qn)
        self.addconfiguration("qz", self._qz)

        self.current_pose = Arm.fkine(self, self._qn)

    def get_ikine(self):
        return Arm.ikine_LM(self.current_pose, mask = [1, 1, 1, 0, 0, 0], slimit = 100, joint_limits=True)