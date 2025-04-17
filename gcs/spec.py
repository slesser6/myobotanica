from dataclasses import dataclass, field
from typing import Callable, List, Optional

from .comms import (
    interactive_move_handler,
    send_command,
    run_script_mode,
)

@dataclass
class CommandSpec:
    menu_label: str
    wire_fmt: str
    wait_time: float = 3.0
    expect_multi: bool = True
    end_keyword: Optional[str] = "END_RESPONSE"
    prompts: List[str] = field(default_factory=list)
    handler: Optional[Callable] = None

EXIT_KEY = "0"


COMMANDS = {
    "1":  CommandSpec("GETSTATE  - Drone status",        "FC:GETSTATE\n"),
    "2":  CommandSpec("ARM       - Arm motors",          "FC:ARM\n", wait_time=20),
    "3":  CommandSpec("TAKEOFF   - Take off to ALT (m)", "FC:TAKEOFF:{0}\n",
                      wait_time=15, prompts=["target altitude (m)"]),
    # ── NEW flexible move ─────────────────────────────────────────────
    "4":  CommandSpec("MOVE (dir/dist/vel/dur)", "", handler=interactive_move_handler),
    "5": CommandSpec("MOVE BACK - Move backward X m",
                     "FC:MOVEBACK:{0}\n",
                     prompts=["distance (m)"]),
    "6": CommandSpec("YAW LEFT  - Yaw left  (deg)",
                     "FC:YAWLEFT:{0}\n",
                     prompts=["degrees"]),
    "7": CommandSpec("YAW RIGHT - Yaw right (deg)",
                     "FC:YAWRIGHT:{0}\n",
                     prompts=["degrees"]),
    "8": CommandSpec("LAND      - Land",
                     "FC:LAND\n", wait_time=10),
    "9": CommandSpec("SERVO     - Two servo angles",
                     "AR:SERVO:PITCH:{0}\nAR:SERVO:PITCH2:{1}\n",
                     prompts=["servo‑1 angle (0‑180)",
                              "servo‑2 angle (0‑180)"]),
    "10": CommandSpec("PUMP ON/OFF",
                      "AR:PUMP:{0}\n",
                      prompts=["ON or OFF"]),
    "11": CommandSpec("Run script",
                      "", handler=lambda s: run_script_mode(s)),
    "12": CommandSpec("Custom cmd",
                      "", prompts=["raw command string"],
                      handler=lambda s, cmd: send_command(
                          s, cmd if cmd.endswith("\n") else cmd + "\n")),
    "13": CommandSpec("RTL       - Return to launch",
                 "FC:RTL\n", wait_time=10),
    "14": CommandSpec("TAKEOFF - AUTO -> 2m",
                    "FC:TAKEOFF\n",
                    wait_time=15),          # same timeout as manual take‑off

    "15": CommandSpec("MOVE FWD - AUTO -> 2m",   "FC:MOVEFWD\n"),
    "16": CommandSpec("MOVE BACK - AUTO -> 2m",   "FC:MOVEBACK\n"),
    "17": CommandSpec("MOVE LEFT - AUTO -> 2m",   "FC:MOVELEFT\n"),   # if you expose it
    "18": CommandSpec("MOVE RIGHT - AUTO -> 2m",   "FC:MOVERIGHT\n"),

    "19": CommandSpec("YAW LEFT  - AUTO -> 15°",
                    "FC:YAWLEFT\n"),

    "20": CommandSpec("YAW RIGHT - AUTO -> 15°",
                    "FC:YAWRIGHT\n"),
    }
