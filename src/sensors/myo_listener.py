import time
import socket
import logging
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
            self._logger.warning("socket already connected")

        self._logger.info(f"Listening on {self._host}:{self._port}")


    def loop(self):
        if not self._enable:
            return
        
        conn, _ = self._sock.accept()
        conn.settimeout(1)
        try:
            data = conn.recv(1024)
        except socket.timeout:
            self._logger.warning("Timeout while waiting for data")
            data = None  # or handle as appropriate
        if not data:
            return
        value = (data.decode())
        self._logger.debug(f"Classification: {value}")
        if (value == "Wrist Rotate In"):
            self.classification = Classification.WRIST_ROT_IN
        elif (value == "Wrist Rotate Out"):
            self.classification = Classification.WRIST_ROT_OUT
        elif (value == "Elbow Flexion"):
            self.classification = Classification.ELBOX_FLEX
        elif (value == "Elbow Extension"):
            self.classification = Classification.ELBOW_EXT
        elif (value == "Wrist Flex In"):
            self.classification = Classification.WRIST_FLEX
        elif (value == "Wrist Extend Out"):
            self.classification = Classification.WRIST_EXT
        elif (value == "Power Grasp"):
            self.classification = Classification.GRASP
        else:
            self.classification = Classification.UNKNOWN
                
    def disconnect(self):
        if not self._enable:
            return
        if self._sock is not None:
            self._sock.close()
            self._sock = None
        
        self._is_initialized = False
        self._logger.info("Closed socket")