"""
SERIAL LINK
-------------
Thin line-oriented wrapper over pyserial for the slave firmware protocol:

  to the board:   "F0,F1,F2,F3,F4,F5,CC\\n"
      F0..F5  degrees 0..180 (at most one decimal), empty field = release servo
      CC      CRC-8 of everything before the last comma, two hex digits

  from the board (6-axis-esp32):
      "BOOT reason=BROWNOUT"
      "STAT rx=120 bad=0 i2c=0 fix=0 reinit=0 spike=0 out=10.0,90.0,90.0,180.0,90.0,-"

The CRC is added here, so the rest of the toolkit only deals in the six fields.
6-axis.ino (Uno) stops reading after six fields and simply ignores it.
"""
import re

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

STAT_COUNTERS = ("rx", "bad", "i2c", "fix", "reinit", "spike")
# `spike` is optional so a board still running the previous firmware keeps reporting.
_STAT_RE = re.compile(r"^STAT " + " ".join(rf"{k}=(\d+)" for k in STAT_COUNTERS[:-1])
                      + r"(?: spike=(\d+))? out=(\S+)$")
_BOOT_RE = re.compile(r"^BOOT reason=(\w+)$")


def crc8(text):
    """CRC-8 (poly 0x07, init 0). Must match crc8() in 6-axis-esp32.ino."""
    crc = 0
    for byte in text.encode("ascii"):
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def frame_line(line):
    """The bytes put on the wire for one command line."""
    return f"{line},{crc8(line):02X}\n".encode("ascii")


def parse_stat_line(line):
    """Returns {"rx", "bad", "i2c", "fix", "reinit", "spike", "out"} or None if `line` isn't
    a well-formed STAT line. `out` holds per-axis output degrees, None = no pulses."""
    m = _STAT_RE.match(line)
    if m is None:
        return None
    stat = {k: int(v or 0) for k, v in zip(STAT_COUNTERS, m.groups())}
    try:
        stat["out"] = [None if f == "-" else float(f) for f in m.group(len(STAT_COUNTERS) + 1).split(",")]
    except ValueError:
        return None
    return stat


def parse_boot_line(line):
    """Returns the reset reason from a BOOT line, or None."""
    m = _BOOT_RE.match(line)
    return m.group(1) if m else None


class SerialLink:
    def __init__(self):
        self.port = None
        self._rx_buf = b""

    @staticmethod
    def available_ports():
        if list_ports is None:
            return []
        return [(p.device, p.description) for p in list_ports.comports()]

    def is_open(self):
        return self.port is not None and self.port.is_open

    def open(self, device, baud):
        self.port = serial.Serial(device, baud, timeout=0, write_timeout=0.2)
        self._rx_buf = b""

    def close(self):
        if self.port is not None:
            try:
                self.port.close()
            finally:
                self.port = None

    def write_line(self, line):
        self.port.write(frame_line(line))

    def read_lines(self):
        n = self.port.in_waiting
        if n:
            self._rx_buf += self.port.read(n)
        lines = []
        while b"\n" in self._rx_buf:
            raw, self._rx_buf = self._rx_buf.split(b"\n", 1)
            text = raw.decode("ascii", errors="replace").strip()
            if text:
                lines.append(text)
        return lines
