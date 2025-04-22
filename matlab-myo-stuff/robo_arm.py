import roboticstoolbox as rtb
from spatialmath import *
import numpy as np
import matplotlib.pyplot as plt
import time
import socket

l = [
    rtb.RevoluteDH(
        d = -2,
        alpha = np.pi/2,
        qlim = [-np.pi/2, np.pi/2]
    ),
    rtb.RevoluteDH(
        a = 3,
    ),
    rtb.RevoluteDH(
        a = 3,
    )
]


bot = rtb.DHRobot(links = l)
q = [0, -np.pi/4, np.pi/4]
pos = bot.fkine(q)
new_pos = pos + SE3(0, 0, -1)
# new_pos = SE3.Rz(np.pi/2) * pos
print(new_pos)
results = bot.ik_LM(Tep = new_pos, mask = [1, 1, 1, 0, 0, 0], joint_limits = True)
print(results)
traj_q = rtb.jtraj(q0 = q, qf = results[0], t = 40)
print(traj_q.q)
bot.plot(q = traj_q.q, dt = .1)

input()
