import roboticstoolbox as rtb
from spatialmath import *
import numpy as np

class DroneArm(rtb.DHRobot):

    def __init__(self):
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
            name="DroneArm",
            manufacturer = "Shadow Wizard Money Gang"
        )

        self.qn = np.array([0, -np.pi/4, np.pi/4]) #nominal/starting pose
        self.qz = np.array([0, 0, 0])

        self.addconfiguration("qn", self.qn)
        self.addconfiguration("qz", self.qz)

        self.current_pose = DroneArm.fkine(self.qn)

    def get_current_pose(self):
        return self.current_pose

    def get_ikine(self, Tep):
        return DroneArm.ikine_LM(Tep, mask = [1, 1, 1, 0, 0, 0], slimit = 100, joint_limits=True)

if __name__ == "__main__":
    drone_arm = DroneArm()
    print(drone_arm)
