"""
SERIAL LINK
-------------
Thin line-oriented wrapper over pyserial for the 6-axis.ino protocol:
"D2,D3,D4,D5,D6,D8\\n" — integer degrees 0..180, empty field = detach.
"""
try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


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
        self.port.write((line + "\n").encode("ascii"))

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
