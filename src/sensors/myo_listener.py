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
        self._conn = None
        self.classification = Classification.UNKNOWN

    def connect(self):
        if not self._enable:
            self._logger.debug("Not enabled")
            return

        if self._conn is None:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(1)
            
            try:
                self._sock.bind((self._host, self._port))
                self._sock.listen(1)
                self._conn = self._sock.accept()
                self._logger.info(f"Listening on {self._host}:{self._port}")
            except:
                self._logger.warning("could not connect to socket.")
        else:
            self._logger.warning("socket already connected.")

    def loop(self):
        if not self._enable:
            return
        
        if self._sock is None or self._conn is None:            
            try:
                self._sock.listen(1)
                self._conn = self._sock.accept()
                self._logger.info(f"Listening on {self._host}:{self._port}")
            except:
                self._logger.warning("Could not connect to socket")
                return
                
        try:
            data = self._conn.recv(1024)
        except:
            self._logger.warning("No data received within timeout")
            return

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

    def disconnect(self):
        if not self._enable:
            return
        if self._sock is not None:
            self._conn.close()
            self._sock.close()
            self._sock = None
        
        self._is_initialized = False
        self._logger.info("Closed socket")