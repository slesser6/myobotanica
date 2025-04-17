from .spec import CommandSpec
from .comms import send_command, interactive_move_handler  # if used
from typing import List

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