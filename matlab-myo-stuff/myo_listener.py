import roboticstoolbox as rtb
from spatialmath import *
import numpy as np
import matplotlib.pyplot as plt
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
        qlim = [-(np.pi),0],
        offset = np.pi/2
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
        s.setblocking(False)
        q = [0, 0, -np.pi/2]
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
                
                print('value', value)
                split = value.split(',')
                print(split)
                value = split[-2]

                # decode what we got and do something with it
                found = True
                if (value == "Wrist Flex In"):
                    print("turning left")
                    # new_position = SE3.Rz(np.pi/16) * position
                elif (value == "Wrist Extend Out"):
                    print("turning right")
                    # new_position = SE3.Rz(-np.pi/16) * position
                # elif (value == "Elbow Flexion"):
                #     q[1] = q[1] + np.pi/50
                # elif (value == "Elbow Extension"):
                #     q[1] = q[1] - np.pi/50
                elif (value == "Wrist Adduction"):
                    print("arm going down")
                    new_position = SE3.Ry(np.pi/8) * position 

                elif (value == "Wrist Abduction"):
                    print("arm going up")
                    new_position = SE3.Ry(-np.pi/8) * position 
                elif (value == "Power Grasp"):
                    print("Spraying water")
                else:
                    found = False
                if found:
                    results = bot.ikine_LM(Tep = new_position, mask = [0,0,0, 1,1,1], slimit=100, joint_limits = True, tol = .1)
                    print(new_position)
                    print(results)
                    if (results.success):
                        print("updating position")
                        traj_q = rtb.jtraj(q0 = q, qf = results.q, t = 20)
                        position = new_position
                        q = results.q
                        print('servo angles', -results.q[1]*180/np.pi, (-results.q[2])*180/np.pi)
                        bot.plot(traj_q.q, dt=0.1, backend = 'pyplot')
                
                # print(position)
                plt.pause(1)
                # time.sleep(4)


                # Add your gimbal control logic here
if __name__ == "__main__":
    plt.figure(1)
    plt.ion()
    start_server()
