"""
gcs_gui.py – quick‑and‑dirty GUI wrapper around your CommandSpec table
----------------------------------------------------------------------
* Built‑in Tkinter (no extra installs)
* Connect / Disconnect controls
* One button per command (auto‑generated)
* When a command needs parameters, a small dialog pops up
* Serial I/O happens in a background thread so the GUI never freezes
"""
import tkinter as tk
from tkinter import ttk, simpledialog, scrolledtext, messagebox
import serial, json, queue, threading
# gcs_gui.py
from gcs.spec   import CommandSpec, COMMANDS   # definition table
from gcs.comms  import send_command            # serial helper


# ─────────────────────── Serial Worker Thread ────────────────────────
class SerialWorker(threading.Thread):
    def __init__(self, ser, rx_queue: queue.Queue):
        super().__init__(daemon=True)
        self.ser = ser
        self.rx_queue = rx_queue
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        while not self._stop.is_set():
            if self.ser.in_waiting:
                self.rx_queue.put(self.ser.readline().decode(errors='ignore'))
            else:
                self._stop.wait(0.05)
                
                
def show_state(self, st: dict) -> None:
    vals = (
        st["mode"],
        st["armed"],
        st["gps"]["sats_visible"],
        st["battery"]["level"],
        f'{st["location"]["lat"]:.5f}',
        f'{st["location"]["lon"]:.5f}',
        f'{st["location"]["alt"]:.1f}',
        f'{st["attitude"]["yaw"]:.0f}°',
    )
    self.tree.item("row", values=vals)
                

# ────────────────────────────   GUI  ─────────────────────────────────
class GCSGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Drone Ground Control Station")

        # --- Connection bar --------------------------------------------------
        bar = ttk.Frame(self)
        bar.pack(fill="x", padx=8, pady=4)

        ttk.Label(bar, text="Port").pack(side="left")
        self.port_var = tk.StringVar(value="COM10")
        ttk.Entry(bar, textvariable=self.port_var, width=8).pack(side="left", padx=(0, 10))

        ttk.Label(bar, text="Baud").pack(side="left")
        self.baud_var = tk.IntVar(value=57600)
        ttk.Entry(bar, textvariable=self.baud_var, width=6).pack(side="left", padx=(0, 10))

        self.connect_btn = ttk.Button(bar, text="Connect", command=self.toggle_serial)
        self.connect_btn.pack(side="left")

        # --- Command buttons -------------------------------------------------
        cmd_frame = ttk.LabelFrame(self, text="Commands")
        cmd_frame.pack(side="left", fill="y", padx=8, pady=4)
        self.cmd_buttons = {}
        for key, spec in COMMANDS.items():
            btn = ttk.Button(cmd_frame, text=f"{key}. {spec.menu_label}",
                             command=lambda k=key: self.handle_command(k),
                             state="disabled")
            btn.pack(fill="x", pady=1)
            self.cmd_buttons[key] = btn

        # --- Console output --------------------------------------------------
        out_frame = ttk.LabelFrame(self, text="Telemetry / Log")
        out_frame.pack(side="right", fill="both", expand=True, padx=8, pady=4)

        self.console = scrolledtext.ScrolledText(out_frame, height=30, state="disabled")
        self.console.pack(fill="both", expand=True)

        # --- Serial objects --------------------------------------------------
        self.ser = None
        self.worker: SerialWorker | None = None
        self.rx_queue: queue.Queue[str] = queue.Queue()
        self.after(100, self._poll_queue)

    # ==========  helpers  ========== #
    def log(self, text: str):
        self.console.configure(state="normal")
        self.console.insert("end", text + "\n")
        self.console.see("end")
        self.console.configure(state="disabled")

    def toggle_serial(self):
        if self.ser and self.ser.is_open:       # already connected -> close
            self.worker.stop()
            self.worker.join()
            self.ser.close()
            self.ser = None
            self.connect_btn.config(text="Connect")
            for b in self.cmd_buttons.values():
                b.config(state="disabled")
            self.log("Disconnected.")
            return

        # try opening
        try:
            self.ser = serial.Serial(
                self.port_var.get(), self.baud_var.get(), timeout=0.1
            )
        except serial.SerialException as e:
            messagebox.showerror("Serial error", str(e))
            return

        self.worker = SerialWorker(self.ser, self.rx_queue)
        self.worker.start()
        self.connect_btn.config(text="Disconnect")
        for b in self.cmd_buttons.values():
            b.config(state="normal")
        self.log(f"Connected on {self.ser.port} @ {self.ser.baudrate}")

    # def _poll_queue(self):
    #     while not self.rx_queue.empty():
    #         self.log(self.rx_queue.get().rstrip())
    #     self.after(100, self._poll_queue)
    
    def _poll_queue(self):
        while not self.rx_queue.empty():
            line = self.rx_queue.get().rstrip()

            if line.startswith("{") and line.endswith("}"):
                # looks like the new JSON state → pretty‑print / render
                try:
                    state = json.loads(line)
                except json.JSONDecodeError:
                    self.log(f"⚠  bad JSON: {line}")
                else:
                    self.show_state(state)      # <‑‑ new helper (see below)
            else:
                # plain text → dump to console
                self.log(line)

        self.after(100, self._poll_queue)    

    # ==========  command handling  ========== #
    def handle_command(self, key: str):
        spec: CommandSpec = COMMANDS[key]

        # Parameter collection (if needed)
        params: list[str] = []
        for prm in spec.prompts:
            val = simpledialog.askstring("Parameter", prm, parent=self)
            if val is None:          # user pressed cancel
                return
            params.append(val.strip())

        # Delegate to custom handler if any
        if spec.handler:
            if params:
                spec.handler(self.ser, spec.wire_fmt.format(*params))
            else:
                spec.handler(self.ser)
            return

        # Otherwise use built‑in sender in a thread
        threading.Thread(
            target=send_command,
            args=(
                self.ser,
                spec.wire_fmt.format(*params),
                spec.wait_time,
                spec.expect_multi,
                spec.end_keyword,
            ),
            daemon=True,
        ).start()

# ──────────────────────────────── run ────────────────────────────────
if __name__ == "__main__":
    GCSGui().mainloop()
