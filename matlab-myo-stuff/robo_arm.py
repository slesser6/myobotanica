import roboticstoolbox as rtb
from spatialmath import *
import numpy as np
import matplotlib.pyplot as plt
import time
import socket

l = [
    rtb.RevoluteDH(
        d = -0.783,
        alpha = np.pi/2,
        a = 6.5,
        # qlim = [-np.pi, np.pi]
    ),
    rtb.RevoluteDH(
        a = 1.833,
        qlim = [-np.pi/2, 0]
    ),
    rtb.RevoluteDH(
        a = 1.81,
    )
]


bot = rtb.DHRobot(links = l)
print(bot)
q = [0, -np.pi/4, np.pi/4]
# pos = bot.fkine(q)
# print(pos)
# new_pos =  SE3(0, 0, 3.5) * pos
# new_pos.t[2] = min(-.1, new_pos.t[2])
# print(new_pos)
# new_pos = SE3.Rz(np.pi/2) * new_pos
# print(new_pos)
# results = bot.ikine_LM(Tep = new_pos, mask = [1, 1, 1, 0, 0, 0], slimit=100, joint_limits = True)
# print(results)
# traj_q = rtb.jtraj(q0 = q, qf = results.q, t = 40)
# # print(traj_q.q)
# bot.plot(q = traj_q.q, dt = .1)
bot.plot([0, 0, 0])

input()
