import time
import socket
import logging
from spatialmath import *
import numpy as np
from src.utils import get_logger
from src.classifier import Classification

class Myoband:
    def __init__(self, cfg):
        self._logger = get_logger("Myoband")
        if cfg.verbose:
            self._logger.setLevel(logging.DEBUG)
        else:
            self._logger.setLevel(logging.INFO)
        self._host = cfg.host
        self._port = cfg.port
        self._enable = cfg.enable
        self._sock = None
        self.classification = Classification.UNKNOWN

    def connect(self):
        if not self._enable:
            self._logger.debug("Not enabled")
            return

        if self._sock is None:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.bind((self._host, self._port))
            self._sock.listen()
        else:
            self._logger.warn("socket already connected")

        self._logger.info(f"Listening on {self._host}:{self._port}")


    def loop(self):
        if not self._enable:
            return
        
        conn, addr = self._sock.accept()
        data = conn.recv(1024)
        if not data:
            return
        value = (data.decode())
        self._logger.debug(f"Classification: {value}")
        if (value == "Wrist Flex In"):
            self.classification = Classification.WRIST_FLEX_TURN_LEFT
        elif (value == "Wrist Extend Out"):
            self.classification = Classification.WRIST_EXT_TURN_RIGHT
        elif (value == "Wrist Adduction"):
            self.classification = Classification.WRIST_ADD_ARM_DOWN
        elif (value == "Wrist Abduction"):
            self.classification = Classification.WRIST_ABD_ARM_UP
        elif (value == "Power Grasp"):
            self.classification = Classification.GRASP_SPRAY
        else:
            self.classification = Classification.UNKNOWN

    def get_pose_modifier(self):
        if(self.classification == Classification.WRIST_FLEX_TURN_LEFT):
            return SE3.Rz(np.pi/16)
        elif(self.classification == Classification.WRIST_EXT_TURN_RIGHT):
            return SE3.Rz(-np.pi/16)
        elif(self.classification == Classification.WRIST_ADD_ARM_DOWN):
            return SE3(0, 0, -.5)
        elif(self.classification == Classification.WRIST_ABD_ARM_UP):
            return SE3(0, 0, .5)
        else:
            return 0
        
    def get_spray(self):
        return (self.classification == Classification.GRASP_SPRAY)
        
    def disconnect(self):
        if not self._enable:
            return
        if self._sock is not None:
            self._sock.close()
            self._sock = None
        
        self._is_initialized = False
        self._logger.info("Closed socket")