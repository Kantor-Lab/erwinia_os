import time
import serial


class SerialSprayer:
    """
    Serial protocol (newline-terminated):
      - 'S'      -> start
      - 'R'      -> stop
      - 'P<ms>'  -> pulse for ms (e.g., P3000)
    Arduino replies with 'OK' or 'ERR' (newline-terminated).
    """

    def __init__(self, port: str, baud: int = 115200, timeout_s: float = 1.0):
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=timeout_s,
            write_timeout=timeout_s,
        )

        # Many Arduinos reset when the serial port opens
        time.sleep(2.0)

        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except Exception:
            pass

    def _write_line(self, s: str) -> None:
        if not s.endswith("\n"):
            s += "\n"
        self.ser.write(s.encode("utf-8"))
        self.ser.flush()

    def _read_line(self) -> str:
        return self.ser.readline().decode("utf-8", errors="ignore").strip()

    def _send_and_expect_ok(self, cmd: str) -> bool:
        self._write_line(cmd)
        resp = self._read_line()
        return resp == "OK"

    def start(self) -> bool:
        return self._send_and_expect_ok("S")

    def stop(self) -> bool:
        return self._send_and_expect_ok("R")

    def pulse(self, ms: int) -> bool:
        return self._send_and_expect_ok(f"P{int(ms)}")

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass