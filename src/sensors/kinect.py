import ctypes
import _ctypes
import pygame
import sys

class Kinect:
    def __init__(self, cfg):
        self.is_win = cfg.is_win           
        self.pygame = None
        self.kinect = None

    def connect(self):
        if self.is_win:
            from pykinect2.PyKinectV2 import FrameSourceTypes_Body
            from pykinect2 import PyKinectRuntime
            self.pygame.init()
            self.kinect = PyKinectRuntime.PyKinectRuntime(FrameSourceTypes_Body)


    def getPosition(self):
        if self.is_win and self.kinect is not None:
            from pykinect2.PyKinectV2 import JointType_HandLeft, JointType_HandRight
            if self.kinect.has_new_body_frame():
                bodies = self.kinect.get_last_body_frame()

                if bodies is not None:
                    for i in range(0, self.kinect.max_body_count):
                        body = bodies.bodies[i]
                        if not body.is_tracked:
                            continue

                        joints = body.joints

                        left_hand = joints[JointType_HandLeft]
                        right_hand = joints[JointType_HandRight]

                        print(f"[KINECT] Left Hand: x={left_hand.Position.x:.2f}, y={left_hand.Position.y:.2f}, z={left_hand.Position.z:.2f}")
                        print(f"[KINECT] Right Hand: x={right_hand.Position.x:.2f}, y={right_hand.Position.y:.2f}, z={right_hand.Position.z:.2f}")

                        return ((left_hand.Position.x, left_hand.Position.y, left_hand.Position.z), (right_hand.Position.x, right_hand.Position.y, right_hand.Position.z))
        
    def disconnect(self):
        if self.is_win and self.kinect is not None and self.pygame is not None:
            self.kinect.close()
            self.pygame.quit()