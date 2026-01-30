# app.py
import time
import threading
from dataclasses import dataclass
from typing import Optional

import tkinter as tk
from tkinter import ttk, messagebox
import serial.tools.list_ports

import config
from serial_worker import SerialWorker, Telemetry


# ---------------- Program model (absolute time schedule) ----------------

@dataclass
class Step:
    at_s: float        # absolute time since start (seconds)
    target_in: float   # inches


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def adc_to_inches(adc: int) -> float:
    """
    Converts raw ADC to inches using calibrated min/max in config.py
    """
    if config.CLAMP_ADC:
        adc = max(config.ADC_MIN, min(config.ADC_MAX, adc))

    span = config.ADC_MAX - config.ADC_MIN
    if span <= 0:
        return 0.0

    ratio = (adc - config.ADC_MIN) / span
    if config.INVERT_POSITION:
        ratio = 1.0 - ratio

    return ratio * config.STROKE_INCHES


def parse_program(text: str) -> list[Step]:
    """
    Lines:
      time_seconds, position_inches

    Meaning:
      at time = time_seconds from Start, set target = position_inches
      controller holds that target until next timestamp
    """
    steps: list[Step] = []
    for idx, raw in enumerate(text.splitlines(), start=1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue

        if "," in line:
            parts = [p.strip() for p in line.split(",", 1)]
        else:
            parts = line.split()

        if len(parts) != 2:
            raise ValueError(f"Line {idx}: expected 'time, position' but got: {raw}")

        try:
            t_abs = float(parts[0])
            pos = float(parts[1])
        except ValueError:
            raise ValueError(f"Line {idx}: time/position must be numbers: {raw}")

        if t_abs < 0:
            raise ValueError(f"Line {idx}: time must be >= 0: {raw}")

        pos = clamp(pos, config.MIN_INCHES, config.MAX_INCHES)
        steps.append(Step(at_s=t_abs, target_in=pos))

    if not steps:
        raise ValueError("No valid steps found.")

    # strictly increasing timestamps
    steps.sort(key=lambda s: s.at_s)
    for i in range(1, len(steps)):
        if steps[i].at_s <= steps[i - 1].at_s:
            raise ValueError(
                "Program times must be strictly increasing (e.g., 2.0, 8.0, 10.0)."
            )

    return steps


# ---------------- GUI App ----------------

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Linear Actuator Controller (Feedback + Schedule)")
        self.geometry("980x590")

        self.serial = SerialWorker()

        # latest telemetry snapshot
        self._last_telem: Optional[Telemetry] = None

        # program thread state
        self._running = False
        self._run_thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()

        # manual hold state (jog override)
        self._held_cmd: Optional[str] = None
        self._hold_job = None

        # command resend tracking
        self._last_cmd_sent: Optional[str] = None
        self._last_cmd_time = 0.0

        # timer state
        self._program_t0: Optional[float] = None

        self._build_ui()
        self._refresh_ports()
        self.after(config.GUI_POLL_MS, self._poll_serial)
        self.after(100, self._tick_clock)  # update PC clock + program timer

    # ---------------- thread-safe UI helper ----------------

    def _ui_set(self, var: tk.StringVar, value: str):
        self.after(0, var.set, value)

    # ---------------- UI ----------------

    def _build_ui(self):
        top = ttk.Frame(self, padding=10)
        top.pack(fill="x")

        ttk.Label(top, text="Port").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(top, textvariable=self.port_var, width=28, state="readonly")
        self.port_cb.pack(side="left", padx=(6, 10))

        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side="left")
        ttk.Label(top, text="Baud").pack(side="left", padx=(12, 0))
        self.baud_var = tk.StringVar(value=str(config.DEFAULT_BAUD))
        self.baud_cb = ttk.Combobox(
            top, textvariable=self.baud_var, width=10, state="readonly",
            values=["9600", "115200"]
        )
        self.baud_cb.pack(side="left", padx=(6, 10))

        ttk.Button(top, text="Connect", command=self._connect).pack(side="left")
        ttk.Button(top, text="Disconnect", command=self._disconnect).pack(side="left", padx=(6, 0))

        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(top, textvariable=self.status_var).pack(side="right")

        body = ttk.Frame(self, padding=10)
        body.pack(fill="both", expand=True)
        body.columnconfigure(0, weight=2)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(1, weight=1)

        # Program input
        prog_frame = ttk.LabelFrame(body, text="Program: time_seconds, position_inches", padding=10)
        prog_frame.grid(row=0, column=0, rowspan=2, sticky="nsew", padx=(0, 10))
        prog_frame.columnconfigure(0, weight=1)
        prog_frame.rowconfigure(1, weight=1)

        ttk.Label(
            prog_frame,
            text=(
                f"Limits: {config.MIN_INCHES:.2f}..{config.MAX_INCHES:.2f} in | "
                f"Tol: {config.TOL_INCHES:.2f} in | "
                f"Control channel: {config.POSITION_CHANNEL}"
            )
        ).grid(row=0, column=0, sticky="w")

        self.prog_text = tk.Text(prog_frame, height=10)
        self.prog_text.grid(row=1, column=0, sticky="nsew", pady=(8, 8))
        self.prog_text.insert(
            "1.0",
            "# Example (absolute timestamps)\n"
            "2.0, 1.70\n"
            "10.0, 2.20\n"
            "20.0, 1.7\n"
        )

        btns = ttk.Frame(prog_frame)
        btns.grid(row=2, column=0, sticky="ew")
        ttk.Button(btns, text="Start", command=self._start).pack(side="left")
        ttk.Button(btns, text="Stop", command=self._stop).pack(side="left", padx=8)

        self.run_state_var = tk.StringVar(value="Idle")
        ttk.Label(btns, textvariable=self.run_state_var).pack(side="right")

        # Live feedback + timer panel
        live = ttk.LabelFrame(body, text="Live feedback + Timer", padding=10)
        live.grid(row=0, column=1, sticky="nsew")
        live.columnconfigure(1, weight=1)

        self.pc_time_var = tk.StringVar(value="-")
        self.elapsed_var = tk.StringVar(value="-")

        self.dir_var = tk.StringVar(value="-")
        self.pos_var = tk.StringVar(value="-")
        self.adc_var = tk.StringVar(value="-")
        self.mv_var = tk.StringVar(value="-")
        self.age_var = tk.StringVar(value="-")

        r = 0
        ttk.Label(live, text="PC time:").grid(row=r, column=0, sticky="w")
        ttk.Label(live, textvariable=self.pc_time_var).grid(row=r, column=1, sticky="w")
        r += 1
        ttk.Label(live, text="Program t (s):").grid(row=r, column=0, sticky="w")
        ttk.Label(live, textvariable=self.elapsed_var).grid(row=r, column=1, sticky="w")
        r += 1
        ttk.Label(live, text="DIR:").grid(row=r, column=0, sticky="w")
        ttk.Label(live, textvariable=self.dir_var).grid(row=r, column=1, sticky="w")
        r += 1
        ttk.Label(live, text="POS (in):").grid(row=r, column=0, sticky="w")
        ttk.Label(live, textvariable=self.pos_var).grid(row=r, column=1, sticky="w")
        r += 1
        ttk.Label(live, text="ADC:").grid(row=r, column=0, sticky="w")
        ttk.Label(live, textvariable=self.adc_var).grid(row=r, column=1, sticky="w")
        r += 1
        ttk.Label(live, text="mV:").grid(row=r, column=0, sticky="w")
        ttk.Label(live, textvariable=self.mv_var).grid(row=r, column=1, sticky="w")
        r += 1
        ttk.Label(live, text="Telemetry age (s):").grid(row=r, column=0, sticky="w")
        ttk.Label(live, textvariable=self.age_var).grid(row=r, column=1, sticky="w")

        # Current step view
        stepf = ttk.LabelFrame(body, text="Current step", padding=10)
        stepf.grid(row=1, column=1, sticky="nsew", pady=(10, 0))
        stepf.columnconfigure(1, weight=1)

        self.step_target_var = tk.StringVar(value="-")
        self.step_timeleft_var = tk.StringVar(value="-")
        ttk.Label(stepf, text="Target (in):").grid(row=0, column=0, sticky="w")
        ttk.Label(stepf, textvariable=self.step_target_var).grid(row=0, column=1, sticky="w")
        ttk.Label(stepf, text="Time to next (s):").grid(row=1, column=0, sticky="w")
        ttk.Label(stepf, textvariable=self.step_timeleft_var).grid(row=1, column=1, sticky="w")

        # Manual control
        manual = ttk.LabelFrame(body, text="Manual Control (hold to move)", padding=10)
        manual.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        for i in range(3):
            manual.columnconfigure(i, weight=1)

        self.btn_fwr = ttk.Button(manual, text="FWR")
        self.btn_rev = ttk.Button(manual, text="REV")
        self.btn_stop = ttk.Button(manual, text="STOP", command=self._manual_stop)

        self.btn_fwr.grid(row=0, column=0, sticky="ew", padx=6)
        self.btn_rev.grid(row=0, column=1, sticky="ew", padx=6)
        self.btn_stop.grid(row=0, column=2, sticky="ew", padx=6)

        self._bind_momentary(self.btn_fwr, "FWR")
        self._bind_momentary(self.btn_rev, "REV")

    # ---------------- clock / timer ----------------

    def _tick_clock(self):
        # PC clock (local time)
        self.pc_time_var.set(time.strftime("%H:%M:%S"))

        # Program elapsed time (from start)
        if self._program_t0 is not None and self._running:
            self.elapsed_var.set(f"{(time.time() - self._program_t0):.2f}")
        else:
            self.elapsed_var.set("-")

        self.after(100, self._tick_clock)

    # ---------------- Serial / connection ----------------

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _connect(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("Error", "Select a serial port.")
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid baud.")
            return

        try:
            self.serial.connect(port, baud=baud, timeout=config.SERIAL_TIMEOUT_S)
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _disconnect(self):
        self._manual_hold_end()
        self._stop()
        self.serial.disconnect()

    def _send(self, cmd: str):
        try:
            self.serial.send(cmd)
        except Exception as e:
            self.status_var.set(f"Send error: {e}")

    # ---------------- Telemetry / feedback helpers ----------------

    def _telemetry_fresh(self) -> bool:
        if not self._last_telem:
            return False
        return (time.time() - self._last_telem.recv_time) <= config.TELEMETRY_TIMEOUT_S

    def _get_adc_mv(self, t: Telemetry):
        if config.POSITION_CHANNEL.upper() == "P2":
            return t.a2, t.mv2
        return t.a1, t.mv1

    def _get_position_in(self, t: Telemetry) -> float:
        adc, _mv = self._get_adc_mv(t)
        return adc_to_inches(adc)

    def _decide_cmd(self, pos_in: float, target_in: float) -> str:
        err = target_in - pos_in
        if err > config.TOL_INCHES:
            return "FWR"
        if err < -config.TOL_INCHES:
            return "REV"
        return "STOP"

    def _maybe_send_cmd(self, cmd: str):
        now = time.time()
        if cmd != self._last_cmd_sent or (now - self._last_cmd_time) * 1000.0 >= config.COMMAND_REPEAT_MS:
            self._send(cmd)
            self._last_cmd_sent = cmd
            self._last_cmd_time = now

    # ---------------- Manual control (momentary) ----------------

    def _bind_momentary(self, widget, cmd: str):
        widget.bind("<ButtonPress-1>", lambda e, c=cmd: self._manual_hold_start(c))
        widget.bind("<ButtonRelease-1>", lambda e: self._manual_hold_end())
        widget.bind("<Leave>", lambda e: self._manual_hold_end())

    def _manual_hold_start(self, cmd: str):
        self._held_cmd = cmd
        self._send(cmd)
        self._schedule_manual_repeat()

    def _schedule_manual_repeat(self):
        if self._held_cmd is None:
            return
        self._hold_job = self.after(config.COMMAND_REPEAT_MS, self._manual_repeat_tick)

    def _manual_repeat_tick(self):
        if self._held_cmd:
            self._send(self._held_cmd)
            self._schedule_manual_repeat()

    def _manual_hold_end(self):
        if self._hold_job:
            try:
                self.after_cancel(self._hold_job)
            except Exception:
                pass
            self._hold_job = None

        if self._held_cmd:
            self._held_cmd = None
            self._send("STOP")

    def _manual_stop(self):
        # hard stop cancels program
        self._stop_evt.set()
        self._running = False
        self._program_t0 = None
        self._send("STOP")
        self._ui_set(self.run_state_var, "Stopped (manual)")
        self._ui_set(self.step_target_var, "-")
        self._ui_set(self.step_timeleft_var, "-")

    # ---------------- Program run (feedback control with absolute timestamps) ----------------

    def _start(self):
        if self._running:
            return

        if config.REQUIRE_TELEMETRY_BEFORE_START and not self._telemetry_fresh():
            messagebox.showerror("Error", "No fresh telemetry. Connect and wait for readings first.")
            return

        try:
            steps = parse_program(self.prog_text.get("1.0", "end").strip())
        except Exception as e:
            messagebox.showerror("Program error", str(e))
            return

        self._stop_evt.clear()
        self._running = True
        self._program_t0 = time.time()
        self._ui_set(self.run_state_var, "Running")

        self._run_thread = threading.Thread(target=self._run_program, args=(steps,), daemon=True)
        self._run_thread.start()

    def _stop(self):
        self._stop_evt.set()
        self._running = False
        self._program_t0 = None
        self._send("STOP")
        self._ui_set(self.run_state_var, "Idle")
        self._ui_set(self.step_target_var, "-")
        self._ui_set(self.step_timeleft_var, "-")

    def _run_program(self, steps: list[Step]):
        self._send("STOP")
        time.sleep(0.05)

        t0 = self._program_t0 if self._program_t0 is not None else time.time()
        i = 0
        current_target: Optional[float] = None
        last_at = steps[-1].at_s

        while not self._stop_evt.is_set():
            now = time.time()
            elapsed = now - t0

            # Manual override active: program pauses output until released
            if self._held_cmd is not None:
                time.sleep(0.02)
                continue

            # Advance schedule if we've reached the next timestamp(s)
            while i < len(steps) and elapsed >= steps[i].at_s:
                current_target = steps[i].target_in
                i += 1
                self._ui_set(self.step_target_var, f"{current_target:.2f}")

            # Before first scheduled time: STOP
            if current_target is None:
                self._maybe_send_cmd("STOP")
                self._ui_set(
                    self.step_timeleft_var,
                    f"{max(0.0, steps[0].at_s - elapsed):.2f} (to first move)"
                )
                time.sleep(0.02)
                continue

            # Telemetry safety
            if not self._telemetry_fresh() or self._last_telem is None:
                self._send("STOP")
                self._ui_set(self.run_state_var, "Telemetry timeout -> STOP")
                time.sleep(0.05)
                continue

            # Feedback control to current target
            pos = self._get_position_in(self._last_telem)
            cmd = self._decide_cmd(pos, current_target)
            self._maybe_send_cmd(cmd)

            # UI: time to next scheduled change
            if i < len(steps):
                self._ui_set(self.step_timeleft_var, f"{max(0.0, steps[i].at_s - elapsed):.2f} (to next)")
            else:
                self._ui_set(self.step_timeleft_var, "last target")

            # End condition: after last timestamp, once within tolerance -> STOP and finish
            if elapsed >= last_at and abs(current_target - pos) <= config.TOL_INCHES:
                self._send("STOP")
                break

            time.sleep(0.02)

        # cleanup
        self._send("STOP")
        self._running = False
        self._program_t0 = None
        self._ui_set(self.run_state_var, "Idle")
        self._ui_set(self.step_target_var, "-")
        self._ui_set(self.step_timeleft_var, "-")

    # ---------------- GUI polling (runs in Tk thread) ----------------

    def _poll_serial(self):
        while not self.serial.status_q.empty():
            self.status_var.set(self.serial.status_q.get())

        last = None
        while not self.serial.telemetry_q.empty():
            last = self.serial.telemetry_q.get()

        if last:
            self._last_telem = last
            self._update_live(last)

        self.after(config.GUI_POLL_MS, self._poll_serial)

    def _update_live(self, t: Telemetry):
        self.dir_var.set(t.direction)

        pos = self._get_position_in(t)
        adc, mv = self._get_adc_mv(t)

        self.pos_var.set(f"{pos:.2f} in (ADC {adc})")
        self.adc_var.set(str(adc))
        self.mv_var.set(str(mv))
        self.age_var.set(f"{(time.time() - t.recv_time):.2f}")

    def on_close(self):
        self._manual_hold_end()
        self._stop()
        self.serial.disconnect()
        self.destroy()


if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
