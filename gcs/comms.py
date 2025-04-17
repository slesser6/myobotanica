import time, serial
# from .spec import CommandSpec   # (if you need it inside interactive_move_handler)
# Optional: keep it only for type‑checking
from typing import TYPE_CHECKING
if TYPE_CHECKING:           # this block is ignored at runtime
    from .spec import CommandSpec


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
