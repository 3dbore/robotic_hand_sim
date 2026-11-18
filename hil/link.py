"""
Serial link to slave_code/6-axis-esp32-twoway-tmc2208 (stream mode only).

PC → board, one line per frame:
    "F0,F1,F2,F3,F4,F5,CC\\n"
    F0 Gripper | F1 Wrist Yaw | F2 Wrist Pitch | F3 Arm B | F4 Arm A | F5 Base
    degrees with at most one decimal; an empty field releases that servo.
    CC = CRC-8 (poly 0x07, init 0) of everything before the last comma, 2 hex.

Board → PC:
    "BOOT reason=..."                                  once after a reset
    "STAT rx= bad= i2c= fix= reinit= spike= out=a,b,c,d,e,f"   every second
"""
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional

import serial
import serial.tools.list_ports
from PyQt5.QtCore import QObject, pyqtSignal

from .config import SERIAL_BAUD


def crc8(text: str) -> int:
    crc = 0
    for b in text.encode("ascii"):
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _fmt(v):
    if v is None:
        return ""
    s = f"{round(v, 1):.1f}"
    return s[:-2] if s.endswith(".0") else s


def make_frame(values) -> str:
    """values: six wire angles (or None to release), in field order."""
    payload = ",".join(_fmt(v) for v in values)
    return f"{payload},{crc8(payload):02X}\n"


@dataclass
class Stat:
    rx: int = 0
    bad: int = 0
    i2c: int = 0
    fix: int = 0
    reinit: int = 0
    spike: int = 0
    out: List[Optional[float]] = field(default_factory=lambda: [None] * 6)


def parse_stat(line: str) -> Optional[Stat]:
    if not line.startswith("STAT "):
        return None
    st = Stat()
    try:
        for tok in line[5:].split():
            key, _, val = tok.partition("=")
            if key == "out":
                parts = val.split(",")
                if len(parts) != 6:
                    return None
                st.out = [None if p in ("-", "") else float(p) for p in parts]
            elif hasattr(st, key):
                setattr(st, key, int(val))
    except ValueError:
        return None
    return st


def list_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


class Link(QObject):
    line = pyqtSignal(str)          # every line from the board
    stat = pyqtSignal(object)       # parsed Stat
    boot = pyqtSignal(str)          # reset reason
    closed = pyqtSignal(str)        # link lost (reason)

    def __init__(self):
        super().__init__()
        self.port = None
        self._thread = None
        self._stop = threading.Event()
        self.sent = 0

    @property
    def is_open(self):
        return self.port is not None and self.port.is_open

    def open(self, device):
        self.close()
        self.port = serial.Serial(device, SERIAL_BAUD, timeout=0.1, write_timeout=0.5)
        self._stop.clear()
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)
            self._thread = None
        if self.port is not None:
            try:
                self.port.close()
            except serial.SerialException:
                pass
            self.port = None

    def send(self, text: str):
        if not self.is_open:
            return False
        try:
            self.port.write(text.encode("ascii"))
            self.sent += 1
            return True
        except (serial.SerialException, OSError) as e:
            self.close()
            self.closed.emit(str(e))
            return False

    def _reader(self):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self.port.read(256)
            except (serial.SerialException, OSError, TypeError, AttributeError) as e:
                if not self._stop.is_set():
                    self.closed.emit(str(e))
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                text = raw.decode("ascii", "replace").strip()
                if not text:
                    continue
                self.line.emit(text)
                st = parse_stat(text)
                if st is not None:
                    self.stat.emit(st)
                elif text.startswith("BOOT"):
                    self.boot.emit(text.partition("reason=")[2] or "?")
            if len(buf) > 4096:
                buf = b""
            time.sleep(0)
