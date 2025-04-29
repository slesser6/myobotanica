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
        qlim = [-np.pi/2 - .2, 0]
    ),
    rtb.RevoluteDH(
        a = 1.81,
        qlim = [-(np.pi)/2,(np.pi)/2],
        offset = np.pi/2
    )
]


bot = rtb.DHRobot(links = l)
print(bot)
q = [0, 0, -np.pi/2]
pos = bot.fkine(q)
print(pos)
# new_pos =  SE3(0, -.5, -2) * pos
new_pos = SE3.Ry(np.pi/3) * pos
# new_pos.t[2] = min(-.1, new_pos.t[2])
print(new_pos)
# new_pos = SE3.Rz(np.pi/2) * new_pos
print(new_pos)
results = bot.ikine_LM(Tep = new_pos, mask = [0, 0, 0, 1, 1, 1], slimit=100, joint_limits = True, tol = .1)
print(results)
traj_q = rtb.jtraj(q0 = q, qf = results.q, t = 40)
# print(traj_q.q)
bot.plot(q = traj_q.q, dt = .1)
print('servo angles', -results.q[1]*180/np.pi, (-results.q[2])*180/np.pi)
# bot.plot(q)

print('servo angles', -q[1]*180/np.pi, (-q[2])*180/np.pi)

input()
