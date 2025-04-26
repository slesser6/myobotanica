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
                d = -0.783,
                alpha = np.pi/2,
                a = 6.5,
                # qlim = [-np.pi, np.pi]
            ),
            rtb.RevoluteDH(
                a = 1.833,
                # qlim = [-np.pi/2, 0]
            ),
            rtb.RevoluteDH(
                a = 1.81,
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
        self._previous_q = self._qn
        self.current_pose = Arm.fkine(self, self._qn)

    def get_ikine(self):
        results = Arm.ikine_LM(self, Tep=self.current_pose, mask = [1, 1, 1, 0, 0, 0], slimit=100, joint_limits = True)
        if (results.success):
            self._previous_q = results.q
            self._logger.debug(f"Servo positions: {results.q}")
            return results.q
        else:
            self._logger.warning("Unable to calculate inverse kinematics.")
            return self._previous_q