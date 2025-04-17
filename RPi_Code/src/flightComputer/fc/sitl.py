"""
flightComputer.fc.sitl
Utility to start / stop a DroneKit‑SITL instance.

Only FlightController should import this; nobody else needs to know.
"""
from __future__ import annotations
import logging, pathlib, tempfile, time
from dronekit_sitl import SITL

_log = logging.getLogger(__name__)

_DEFAULT_BIN = pathlib.Path.home() / ".dronekit/sitl/copter-3.3/apm"
_DEFAULT_HOME = "37.8719,-122.2585,100,0"      # Berkeley, 100 m AGL

class SitlManager:
    def __init__(self,
                 binary: str | pathlib.Path = _DEFAULT_BIN,
                 home_str: str = _DEFAULT_HOME):
        self._binary = str(binary)
        self._home   = home_str
        self._sitl   = None                     # type: SITL | None

    # ------------------------------------------------------------------
    def start(self) -> str:
        """Launch SITL and return its connection string (e.g. 'tcp:127.0.0.1:5760')."""
        if self._sitl:
            return self._sitl.connection_string()

        _log.info("Starting SITL with home %s …", self._home)
        self._sitl = SITL(path=self._binary)
        work_dir = tempfile.mkdtemp(prefix="fc_sitl_")
        self._sitl.launch(["--home", self._home],
                          await_ready=True, restart=True, wd=work_dir)
        # small delay so ArduPilot finishes boot
        time.sleep(1)
        conn = self._sitl.connection_string()
        _log.info("SITL up @ %s", conn)
        return conn

    # ------------------------------------------------------------------
    def stop(self):
        if self._sitl:
            _log.info("Stopping SITL …")
            self._sitl.stop()
            self._sitl = None
