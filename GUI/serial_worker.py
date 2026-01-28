# serial_worker.py
import time
import threading
import queue
import re
from dataclasses import dataclass
from typing import Optional

import serial


TELEM_RE = re.compile(
    r"^T,(?P<dir>FWR|REV|STOP),"
    r"A1=(?P<a1>\d+),IN1=(?P<in1>-?\d+),MV1=(?P<mv1>\d+),"
    r"A2=(?P<a2>\d+),IN2=(?P<in2>-?\d+),MV2=(?P<mv2>\d+)\s*$"
)


@dataclass
class Telemetry:
    direction: str
    a1: int
    in1_x100: int
    mv1: int
    a2: int
    in2_x100: int
    mv2: int
    recv_time: float  # time.time() when received

    @staticmethod
    def from_line(line: str, recv_time: float) -> Optional["Telemetry"]:
        m = TELEM_RE.match(line)
        if not m:
            return None
        return Telemetry(
            direction=m.group("dir"),
            a1=int(m.group("a1")),
            in1_x100=int(m.group("in1")),
            mv1=int(m.group("mv1")),
            a2=int(m.group("a2")),
            in2_x100=int(m.group("in2")),
            mv2=int(m.group("mv2")),
            recv_time=recv_time,
        )


class SerialWorker:
    """
    Reusable serial + telemetry worker.
    - connect/disconnect
    - thread-safe send()
    - background read + parse -> telemetry_q
    """
    def __init__(self):
        self.ser: Optional[serial.Serial] = None
        self.thread: Optional[threading.Thread] = None
        self.stop_evt = threading.Event()

        self.telemetry_q: "queue.Queue[Telemetry]" = queue.Queue()
        self.status_q: "queue.Queue[str]" = queue.Queue()

        self._lock = threading.Lock()

    def connect(self, port: str, baud: int = 9600, timeout: float = 0.2):
        self.disconnect()
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        except Exception as e:
            raise RuntimeError(f"Failed to open {port}: {e}") from e

        self.stop_evt.clear()
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()

        self.status_q.put(f"Connected: {port} @ {baud}")
        time.sleep(1.2)  # Arduino reset on open (common)
        self.flush()

    def disconnect(self):
        self.stop_evt.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.thread = None

        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.status_q.put("Disconnected")

    def flush(self):
        with self._lock:
            if self.ser:
                try:
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                except Exception:
                    pass

    def send(self, cmd: str):
        with self._lock:
            if not self.ser:
                raise RuntimeError("Serial not connected")
            self.ser.write((cmd.strip() + "\n").encode("ascii", errors="ignore"))

    def _reader_loop(self):
        while not self.stop_evt.is_set():
            if not self.ser:
                time.sleep(0.05)
                continue
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                now = time.time()
                t = Telemetry.from_line(line, recv_time=now)
                if t:
                    self.telemetry_q.put(t)

            except Exception as e:
                self.status_q.put(f"Serial error: {e}")
                time.sleep(0.2)
