import roboticstoolbox as rtb
import spatialmath as SE3
import numpy as np
import matplotlib.pyplot as plt
import matlab.engine
import time
import socket



l = [
    rtb.RevoluteDH(
        d = -4,
        alpha = np.pi/2
    ),
    rtb.RevoluteDH(
        a = 4,
    ),
    rtb.RevoluteDH(
        a = 4,
    )
]


bot = rtb.DHRobot(links = l)
q = [0, 0, 0]

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
                if (value == "Wrist Flex In"):
                    q[0] = q[0] - np.pi/20
                elif (value == "Wrist Extend Out"):
                    q[0] = q[0] + np.pi/20
                elif (value == "Elbow Flexion"):
                    q[1] = q[1] - np.pi/20
                elif (value == "Elbow Extension"):
                    q[1] = q[1] + np.pi/20
                elif (value == "Wrist Adduction"):
                    q[2] = q[2] - np.pi/20
                elif (value == "Wrist Abduction"):
                    q[2] = q[2] + np.pi/20
                elif (value == "Power Grasp"):
                    print("Spraying water")
                else:
                    found = False
                if found:
                    bot.plot(q)
                



                # Add your gimbal control logic here
if __name__ == "__main__":
    plt.ion()
    bot.plot(q)
    start_server()
