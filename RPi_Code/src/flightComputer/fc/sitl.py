import os
import subprocess
import tempfile
import pathlib
import logging
import time
from typing import Union

_log = logging.getLogger(__name__)

# ── default location *unless* the user sets $ARDUPILOT_SITL_BIN ─────────
_DEFAULT_BIN = pathlib.Path(
    os.environ.get(
        "ARDUPILOT_SITL_BIN",
        pathlib.Path.home() / "ardupilot" / "build" / "sitl" / "bin" / "arducopter"
    )
).expanduser()

class SitlManager:
    """
    Thin wrapper around the ArduCopter SITL binary.
    Spawns the process, checks it stayed up, returns the MAVSDK URL,
    and can be shut down later via `stop()`.
    """
    def __init__(self, binary: Union[str, pathlib.Path] = _DEFAULT_BIN):
        self._bin  = pathlib.Path(binary).expanduser()
        self._proc: subprocess.Popen | None = None

        if not self._bin.exists():
            raise FileNotFoundError(
                f"SITL binary not found at '{self._bin}'.\n"
                "• Build ArduPilot with  `./waf copter`  or\n"
                "• set ARDUPILOT_SITL_BIN=/path/to/arducopter"
            )

    # ------------------------------------------------------------------
    def start(self, udp_port: int = 14540) -> str:
        if self._proc:
            return f"udp://:{udp_port}"

        workdir = tempfile.mkdtemp(prefix="sitl_")

        # ---- write FRAME_CLASS / FRAME_TYPE into a tiny defaults file ----
        defaults = pathlib.Path(workdir) / "frame.par"
        defaults.write_text(
            "FRAME_CLASS,1\n"      # 1 = Copter
            "FRAME_TYPE,1\n"       # 1 = X-layout
            "ARMING_CHECK,0\n"     # optional: disable pre-arm checks
        )
        # -----------------------------------------------------------------

        cmd = [
            str(self._bin), "-I", "0",
            "--model", "quad",
            "--speedup", "1",
            "--serial0", f"udpclient:127.0.0.1:{udp_port}",
            "--defaults", str(defaults),        # <── new
        ]
        _log.info("Launching SITL: %s", " ".join(cmd))

        self._proc = subprocess.Popen(cmd, cwd=workdir)

        # ── safety: fail fast if the binary crashes immediately ───────
        t0 = time.time()
        while time.time() - t0 < 2.0:
            if self._proc.poll() is not None:        # process exited
                rc = self._proc.returncode
                raise RuntimeError(
                    f"SITL exited early with code {rc}. "
                    f"Command was: {' '.join(cmd)}"
                )
            time.sleep(0.1)

        return f"udp://:{udp_port}"  

    # ------------------------------------------------------------------
    def stop(self):
        """Terminate the SITL process (if it is running)."""
        if not self._proc:
            return

        self._proc.terminate()
        try:
            self._proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self._proc.kill()

        _log.info("SITL stopped with exit code %s", self._proc.returncode)
        self._proc = None
