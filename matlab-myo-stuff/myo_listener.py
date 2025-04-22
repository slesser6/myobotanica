import roboticstoolbox as rtb
from spatialmath import *
import numpy as np
import matplotlib.pyplot as plt
import matlab.engine
import time
import socket



l = [
    rtb.RevoluteDH(
        d = -2,
        alpha = np.pi/2,
        qlim = [-np.pi, np.pi]
    ),
    rtb.RevoluteDH(
        a = 3,
    ),
    rtb.RevoluteDH(
        a = 3,
    )
]


bot = rtb.DHRobot(links = l)


# matlabCode = input("Enter the matlab engine name:")

# eng = matlab.engine.connect_matlab(matlabCode)
# try:
#     while True:
#         test = eng.workspace['className']
#         print(test)
#         time.sleep(0.25)
# except KeyboardInterrupt:
#     print("exiting program")
#     exit()
def start_server(host='localhost', port=65432):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        print(f"Server started at {host}:{port}")
        conn, addr = s.accept()
        q = [0, -np.pi/4, np.pi/4]
        position = bot.fkine(q) #initial position
        new_position = position
        with conn:
            print(f"Connected by {addr}")
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                # Process the received data
                value = (data.decode())
                print(value)

                # decode what we got and do something with it
                found = True
                if (value == "Wrist Rotate In"):
                    print("turning right")
                    new_position = SE3.Rz(np.pi/8) * position
                elif (value == "Wrist Rotate Out"):
                    print("turning left")
                    new_position = SE3.Rz(-np.pi/8) * position
                # elif (value == "Elbow Flexion"):
                #     q[1] = q[1] + np.pi/50
                # elif (value == "Elbow Extension"):
                #     q[1] = q[1] - np.pi/50
                elif (value == "Wrist Flex In"):
                    print("arm going down")
                    new_position = position + SE3(0, 0, -.25)
                elif (value == "Wrist Extend Out"):
                    print("arm going up")
                    new_position = position + SE3(0, 0, .25)
                elif (value == "Power Grasp"):
                    print("Spraying water")
                else:
                    found = False
                if found:
                    results = bot.ik_LM(Tep = new_position, mask = [1, 1, 1, 0, 0, 0], joint_limits = True)
                    print(new_position)
                    print(results)
                    if (results[1]):
                        print("updating position")
                        traj_q = rtb.jtraj(q0 = q, qf = results[0], t = 20)
                        position = new_position
                        q = results[0]
                        bot.plot(traj_q.q, dt=0.1, backend = 'pyplot')
                
                print(position)
                plt.pause(2)
                # time.sleep(4)


                # Add your gimbal control logic here
if __name__ == "__main__":
    plt.figure(1)
    plt.ion()
    start_server()
