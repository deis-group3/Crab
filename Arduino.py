# arduino_api.py

import serial
from dataclasses import dataclass

@dataclass
class ArduinoStatus:
    dist: float | None   # None if invalid
    gas: int             # 0 or 1
    light: int           # 0 or 1
    claw: str            # "OPEN" or "CLOSED" (or whatever Arduino sends)


class ArduinoClawInterface:
    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 9600, timeout: float = 1.0):
        """
        Initialize serial connection to Arduino.
        """
        self.ser = serial.Serial(port, baudrate, timeout=timeout)

    def _parse_line(self, line: str) -> ArduinoStatus | None:
        """
        Parse a line of the form:
        DIST:7.3;GAS:0;LIGHT:1;CLAW:OPEN
        Returns ArduinoStatus or None if parsing fails.
        """
        line = line.strip()
        if not line:
            return None

        parts = line.split(";")
        data = {}
        try:
            for p in parts:
                key, value = p.split(":", 1)
                data[key] = value

            # Convert to proper types with safe defaults
            dist = float(data.get("DIST", "-1")) if data.get("DIST") not in (None, "") else -1
            gas = int(data.get("GAS", "0"))
            light = int(data.get("LIGHT", "0"))
            claw = data.get("CLAW", "UNKNOWN")

            # Dist < 0 means invalid, map to None
            if dist < 0:
                dist_val = None
            else:
                dist_val = dist

            return ArduinoStatus(
                dist=dist_val,
                gas=gas,
                light=light,
                claw=claw
            )
        except Exception:
            return None

    def read_status(self) -> ArduinoStatus | None:
        """
        Read one line from Arduino and return an ArduinoStatus.
        Returns None if no valid line was received.
        """
        line_bytes = self.ser.readline()
        if not line_bytes:
            return None

        try:
            line = line_bytes.decode("utf-8", errors="ignore")
        except UnicodeDecodeError:
            return None

        return self._parse_line(line)

    def close(self):
        """
        Close the serial connection.
        """
        if self.ser and self.ser.is_open:
            self.ser.close()
