from __future__ import annotations
import serial
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional
from typing import Dict          # add at the top

def interactive_move_handler(ser):
    """
    Prompt the user for direction and (optional) distance / velocity / duration,
    then build a 'FC:MOVE:…' message and send it.
    Blank entries fall back to the FC defaults.
    """
    # Direction (mandatory)
    dir_ = input("Direction [FWD/BACK/LEFT/RIGHT/UP/DOWN]: ").strip().upper()
    if dir_ not in {"FWD", "BACK", "LEFT", "RIGHT", "UP", "DOWN"}:
        print("❌  Unknown direction.")
        return

    # Optional numeric fields (blank => default on FC side)
    dist      = input("Distance [m]      (Enter for default): ").strip()
    velocity  = input("Velocity [m/s]    (Enter for default): ").strip()
    duration  = input("Duration [sec]    (Enter for auto ‑‑ distance/vel): ").strip()

    # Build the colon‑separated payload, preserving empty fields
    payload = f"FC:MOVE:{dir_}:{dist}:{velocity}:{duration}\n"
    # Collapse any trailing ':' characters the user left blank
    payload = payload.rstrip(":") + "\n"

    send_command(ser, payload, wait_time=5, expect_multi=True, end_keyword="END_RESPONSE")

@dataclass
class CommandSpec:
    menu_label: str                  # How it appears in the menu
    wire_fmt: str                    # The bytes we’ll actually send
    wait_time: float = 3.0
    expect_multi: bool = True
    end_keyword: Optional[str] = "END_RESPONSE"
    # Optional interactive prompts, e.g. ["altitude (m)", "yaw deg"]
    prompts: List[str] = field(default_factory=list)
    # Optional completely custom handler (rarely needed)
    handler: Optional[Callable] = None

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
    "14": CommandSpec("TAKEOFF  AUTO -> 2m",
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

EXIT_KEY = "0"

def print_menu(cmds, exit_key="0", exit_label="Exit"):
    """
    Pretty‑prints a table of commands.
    The first row is always the exit option.
    """
    # ---------- build the list of rows ---------------------------------
    rows = [(exit_key, exit_label, "")]            # ← exit row first
    for key, spec in cmds.items():
        label = spec.menu_label.strip()

        if " - " in label:                         #  "CMD  - desc"
            cmd, desc = label.split(" - ", 1)
        elif " (" in label:                        #  "CMD (desc)"
            cmd, desc = label.split(" (", 1)
            desc = "(" + desc
        else:
            cmd, desc = label, ""

        rows.append((key, cmd.strip(), desc.strip()))

    # ---------- column widths ------------------------------------------
    w_id  = max(len(r[0]) for r in rows)
    w_cmd = max(len(r[1]) for r in rows)
    bar   = "─" * (w_id + w_cmd + 25)

    # ---------- print ---------------------------------------------------
    print("\n=== Drone Ground Station ===")
    print(bar)
    print(f"{'#':<{w_id}}  {'Command':<{w_cmd}}  Description")
    print(bar)
    for r in rows:
        print(f"{r[0]:<{w_id}}  {r[1]:<{w_cmd}}  {r[2]}")
    print(bar)



def send_command(ser, command, wait_time=2, expect_multi=False, end_keyword=None):
    """
    Sends a command to the RPi and prints all responses until either wait_time expires
    or an end_keyword is detected.
    
    Args:
        ser (serial.Serial): The open serial connection to the RPi.
        command (str): The command string to send (with trailing newline).
        wait_time (float): Maximum seconds to wait if no end_keyword is detected.
        expect_multi (bool): If True, continues reading lines until wait_time expires.
        end_keyword (str): If provided, reading stops when a line exactly matches this keyword.
    
    Returns:
        List[str]: A list of response lines received (excluding the end marker).
    """
    # Flush any stale data before sending
    ser.reset_input_buffer()
    print(f"\nSending command to RPi: {command.strip()}")
    ser.write(command.encode())

    responses = []
    end_time = time.time() + wait_time
    while True:
        if ser.in_waiting:
            line = ser.readline().decode().strip()
            # Check if we've reached the end marker
            if end_keyword and line == end_keyword:
                break
            print(line)
            responses.append(line)
            # If not expecting multi-line responses, break after one line.
            if not expect_multi:
                break
        # If no data is arriving and we've passed the wait_time, break out.
        if time.time() > end_time:
            break
        time.sleep(0.1)
    if not responses:
        print("No response (timeout).")
    return responses

def run_script_mode(ser):
    """
    Example script mode: sends a sequence of commands.
    Each command on the RPi side should append an end-of-response marker.
    """
    print("\nRunning preconfigured script...")
    # Each tuple: (command, wait_time, end_keyword)
    script_commands = [
        ("FC:ARM\n", 10, "END_RESPONSE"),
        ("FC:TAKEOFF:5\n", 15, "END_RESPONSE"),
        ("FC:MOVE:FWD\n", 5, "END_RESPONSE"),
        ("FC:LAND\n", 10, "END_RESPONSE"),
    ]
    for cmd, wait_sec, end_kw in script_commands:
        send_command(ser, cmd, wait_time=wait_sec, expect_multi=True, end_keyword=end_kw)
        time.sleep(2)
    print("Script mode complete.")


def prompt_user(prompts: List[str]) -> List[str]:
    return [input(f"{p}: ").strip() for p in prompts]

def execute_command(ser, spec: CommandSpec):
    # Completely custom handler?  Delegate and return.
    if spec.handler and not spec.prompts:
        spec.handler(ser)
        return

    # Get any user parameters
    params = prompt_user(spec.prompts) if spec.prompts else []
    # If the spec *also* has a handler, give it the formatted command
    if spec.handler:
        formatted = spec.wire_fmt.format(*params)
        spec.handler(ser, formatted)
        return
    # Otherwise use the built‑in sender
    send_command(
        ser,
        spec.wire_fmt.format(*params),
        wait_time=spec.wait_time,
        expect_multi=spec.expect_multi,
        end_keyword=spec.end_keyword
    )

def menu_loop(ser):
    while True:
        print_menu(COMMANDS, EXIT_KEY)          # ← one call does it all
        # print_menu(COMMANDS)          # ← one call does it all
        choice = input("Choose: ").strip()
        if choice == EXIT_KEY:
            break
        if choice not in COMMANDS:
            print("Invalid.")
            continue
        execute_command(ser, COMMANDS[choice])


def main():
    try:
        with serial.Serial("COM10", 57600, timeout=0.5) as ser:
            print("Serial up.  Waiting 2 s for radio…")
            time.sleep(2)
            menu_loop(ser)
    except serial.SerialException as e:
        print(f"Serial error: {e}")


if __name__ == "__main__":
    main()

