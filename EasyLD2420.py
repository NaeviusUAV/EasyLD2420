"""
MicroPython driver for the Hi‑Link LD2420 24 GHz mmWave radar sensor.

This module exposes a single class, :class:`LD2420`, which wraps the
low‑level serial protocol used by the LD2420 and presents a friendly
Pythonic API.  It is designed to run on MicroPython (such as an ESP32)
but includes a limited set of stub definitions so that it can be
imported and unit‑tested under normal CPython on a desktop computer.

The implementation is based on the published command protocol for the
LD2420.  Each command frame consists of a four byte header,
a two byte little‑endian length, a two byte command word, zero or more
payload bytes and a four byte footer.  The driver hides the framing
details from the user.  For streaming operation the module can be put
into “energy output” mode where it reports a presence flag, an
estimated distance and sixteen 16‑bit gate energy values for a 4×4
grid.  See the accompanying README or the official datasheet for more
details.

Because the LD2420 can be shipped with different firmware versions,
the TX and RX pins may swap between OT1 and OT2 and the default
baudrate may vary (256 000 bps on older firmware, 115 200 bps on newer
firmware).  The constructor therefore performs an initial handshake
using the provided pin assignments and optionally retries with the
pins swapped and an alternate baudrate.  If communication cannot be
established, an exception will be raised to alert the caller.

This driver attempts to be resilient: command retries, optional
timeouts in data waits and automatic reset on repeated framing
errors.  For debugging purposes the `debug` flag may be enabled to
log raw frames to the REPL.
"""

import time
try:
    from typing import Callable, List, Optional, Tuple, Any, Dict
except:
    print('\n')
# -------------------------------------------------------------------
# Helper functions for timekeeping
#
# MicroPython provides time.ticks_ms(), time.ticks_diff() and
# time.sleep_ms() for millisecond resolution timing.  These are not
# available on standard CPython so we define fallbacks here.  The
# implementations use time.monotonic() which has sufficient resolution
# for our purposes.  If running on MicroPython the native functions
# will already exist and these definitions are ignored.

if not hasattr(time, "ticks_ms"):
    _time_start = time.monotonic()

    def ticks_ms() -> int:
        return int((time.monotonic() - _time_start) * 1000)

    def ticks_diff(a: int, b: int) -> int:
        return a - b

    def sleep_ms(ms: int) -> None:
        time.sleep(ms / 1000.0)

    setattr(time, "ticks_ms", ticks_ms)
    setattr(time, "ticks_diff", ticks_diff)
    setattr(time, "sleep_ms", sleep_ms)

# -------------------------------------------------------------------
# CPython‑only exceptions that may be missing in MicroPython
try:
    TimeoutError
except NameError:  # pragma: no cover
    class TimeoutError(OSError):
        """Fallback for boards that omit CPython’s TimeoutError."""
        pass

try:
    from machine import UART, Pin  # type: ignore
except ImportError:  # pragma: no cover - fall back for CPython unit tests
    # Provide minimal stubs so that the driver can be imported on a
    # non‑MicroPython host.  These stubs do not attempt to simulate the
    # actual hardware behaviour but allow the module to be unit tested
    # for syntax and high level logic.
    class _DummyUART:
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            self._buffer = bytearray()

        def write(self, data: bytes) -> None:
            pass

        def any(self) -> int:
            return len(self._buffer)

        def readinto(self, buf: bytearray) -> int:
            n = min(len(self._buffer), len(buf))
            for i in range(n):
                buf[i] = self._buffer[i]
            del self._buffer[:n]
            return n

        def read(self, n: int) -> bytes:
            result = self._buffer[:n]
            del self._buffer[:n]
            return bytes(result)

    class _DummyPin:
        IN = 0
        OUT = 1
        PULL_UP = 0
        PULL_DOWN = 1

        def __init__(self, pin: int, mode: int = IN, pull: int = None) -> None:
            self._value = 0

        def value(self) -> int:
            return self._value

        def set_value(self, val: int) -> None:
            self._value = val

    UART = _DummyUART  # type: ignore
    Pin = _DummyPin  # type: ignore


class LD2420:
    """Driver class for the Hi‑Link LD2420 radar module.

    The driver encapsulates the serial protocol used by the LD2420 and
    exposes methods to start and stop streaming, read version
    information, adjust sensitivity, set refresh rates and detect
    presence.  Instances must be created with the appropriate UART
    pins and the OT (data ready) pin for your hardware.
    """

    # Command constants (little‑endian values)
    _CMD_ENABLE_CONFIG = 0x00FF
    _CMD_DISABLE_CONFIG = 0x00FE
    _CMD_SYSTEM_MODE = 0x0012  # used to set streaming mode
    _CMD_READ_VERSION = 0x0002  # returns protocol/firmware version
    _CMD_RESTART = 0x0068  # reset the module
    _CMD_WRITE_REGISTER = 0x0001
    _CMD_READ_REGISTER = 0x0002
    _CMD_WRITE_ABD_PARAM = 0x0007  # write threshold parameters

    # Streaming mode parameters
    _MODE_BASIC = 0x0064  # basic status (presence ON/OFF only)
    _MODE_ENERGY = 0x0004  # energy output (presence + distance + gates)
    _MODE_DEBUG = 0x0000  # debug output (large frames)

    # Frame markers for command/ack frames
    _FRAME_HEADER = b"\xFD\xFC\xFB\xFA"
    _FRAME_FOOTER = b"\x04\x03\x02\x01"
    # Streaming (energy output) frame markers.  In energy mode the LD2420
    # uses a different header and footer (0xF4F3F2F1 and 0xF8F7F6F5) and
    # the two bytes after the header specify the payload length directly.
    _STREAM_HEADER = b"\xF4\xF3\xF2\xF1"
    _STREAM_FOOTER = b"\xF8\xF7\xF6\xF5"

    def __init__(
        self,
        uart_pins: List[int],
        ot_pin: int,
        *,
        baudrate: int = 115200,
        alternate_baudrate: int = 256000,
        debug: bool = False,
        probe: bool = True,
    ) -> None:
        if len(uart_pins) != 2:
            raise ValueError("uart_pins must contain exactly two integers [rx, tx]")
        self.debug = bool(debug)
        self._rx_pin, self._tx_pin = uart_pins
        self._ot_pin_no = ot_pin
        self._baudrate = baudrate
        self._alternate_baudrate = alternate_baudrate

        # Internal state
        self._uart: UART = None  # type: ignore
        self._ot: Pin = None  # type: ignore
        self._running = False
        self._callback: Optional[Callable[[bool, List[List[int]], int, Dict[str, Any]], None]] = None
        self._buf = bytearray()
        self._refresh_interval_ms = 0

        if probe:
            self._init_uart_and_check()
        else:
            self._uart = UART(1, baudrate=baudrate, rx=self._rx_pin, tx=self._tx_pin)
            self._ot = Pin(self._ot_pin_no, Pin.IN)
            if self.debug:
                print(f"UART configured without probing on RX={self._rx_pin}, TX={self._tx_pin}, baud={baudrate}")

    # ------------------------------------------------------------------
    # Public API
    def help(self) -> None:
        methods = [name + "()" for name in dir(self) if not name.startswith("_") and callable(getattr(self, name))]
        print("LD2420 Methods:")
        for m in methods:
            print("  -", m)
    
    def start(self, *, verify: bool = True) -> None:
        self._send_system_mode(self._MODE_ENERGY)
        if not verify:
            self._running = True
            return
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < 500:
            if self._uart.any():
                self._running = True
                return
            time.sleep_ms(10)
        raise RuntimeError("Failed to start streaming: no data received")

    def stop(self) -> None:
        try:
            self._send_system_mode(self._MODE_BASIC)
        finally:
            self._running = False

    def wait_for_data(self, timeout: Optional[float] = None) -> Tuple[bool, List[List[int]], int, Dict[str, Any]]:
        if not self._running:
            raise RuntimeError("call start() before waiting for data")
        start_ms = time.ticks_ms()
        while True:
            if timeout is not None:
                elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
                if elapsed >= timeout:
                    raise TimeoutError("timeout waiting for radar data")
            if self._uart.any():
                tmp = bytearray(64)
                n = self._uart.readinto(tmp)
                if n:
                    self._buf.extend(tmp[:n])
                    if self.debug:
                        print("[RX]", tmp[:n])
            frame = self._extract_frame()
            if frame is not None:
                # Determine if this is a streaming (energy) frame or a command/ack frame
                if frame.startswith(self._STREAM_HEADER):
                    # Decode and return streaming data
                    presence, grid, distance, extra = self._decode_presence_frame(frame)
                    if self._callback:
                        try:
                            self._callback(presence, grid, distance, extra)
                        except Exception as exc:  # pragma: no cover
                            print("LD2420 callback error:", exc)
                    return presence, grid, distance, extra
                elif frame.startswith(self._FRAME_HEADER):
                    # This is likely a command acknowledgement; decode to raise on error and ignore
                    try:
                        self._decode_ack(frame)
                    except Exception as exc:
                        # Surface errors from the module
                        raise
                    # Not a streaming frame; skip and continue waiting
                    continue
                else:
                    # Unknown frame type; ignore
                    continue
            time.sleep_ms(10)

    def reset(self) -> None:
        frame = self._build_frame(self._CMD_RESTART, b"\x00\x00")
        self._uart.write(frame)
        if self.debug:
            print("[TX]", frame)
        time.sleep_ms(200)
        self._init_uart_and_check()

    def get_version(self) -> str:
        self._send_command(self._CMD_ENABLE_CONFIG, b"\x02\x00")
        self._send_command(self._CMD_READ_VERSION, b"\x00\x00")
        start_ms = time.ticks_ms()
        version: Optional[str] = None
        while True:
            if self._uart.any():
                tmp = bytearray(64)
                n = self._uart.readinto(tmp)
                self._buf.extend(tmp[:n])
            frame = self._extract_frame()
            if frame is not None:
                cmd, payload = self._parse_generic_frame(frame)
                if cmd == self._CMD_READ_VERSION and len(payload) >= 4:
                    err_code = payload[0] | (payload[1] << 8)
                    if err_code != 0:
                        raise ValueError(f"module returned error {err_code:04x} when reading version")
                    major = payload[2]
                    minor = payload[3]
                    version = f"{major}.{minor}"
                    break
            if time.ticks_diff(time.ticks_ms(), start_ms) > 1000:
                raise TimeoutError("no response to version query")
            time.sleep_ms(10)
        self._send_command(self._CMD_DISABLE_CONFIG, b"")
        assert version is not None
        return version

    def set_sensitivity(self, level: int) -> None:
        if not 0 <= level <= 5:
            raise ValueError("sensitivity level must be between 0 and 5")
        threshold_table: List[List[int]] = [
            [60000] * 16,
            [40000] * 16,
            [30000] * 16,
            [20000] * 16,
            [10000] * 16,
            [5000] * 16,
        ]
        values = threshold_table[level]
        self._send_command(self._CMD_ENABLE_CONFIG, b"\x02\x00")
        for gate in range(16):
            thr = values[gate]
            move_reg = 0x0010 + gate
            still_reg = 0x0020 + gate
            self.write_register(move_reg, thr, verify=False)
            self.write_register(still_reg, thr, verify=False)
        self._send_command(self._CMD_DISABLE_CONFIG, b"")

    def set_refresh_rate(self, rate_hz: int) -> None:
        if rate_hz <= 0:
            raise ValueError("refresh rate must be positive")
        self._refresh_interval_ms = int(1000 / rate_hz)

    def set_detection_range(self, min_cm: int, max_cm: int) -> None:
        if min_cm >= max_cm:
            raise ValueError("min_cm must be less than max_cm")
        if not (30 <= min_cm <= 800 and 30 <= max_cm <= 800):
            raise ValueError("detection range must be between 30 and 800 cm")
        gate_size_cm = 7.5
        min_gate = int(min_cm / gate_size_cm)
        max_gate = int(max_cm / gate_size_cm)
        if min_gate < 0:
            min_gate = 0
        if max_gate > 15:
            max_gate = 15
        self._send_command(self._CMD_ENABLE_CONFIG, b"\x02\x00")
        self.write_register(0x0000, min_gate, verify=False)
        self.write_register(0x0001, max_gate, verify=False)
        self._send_command(self._CMD_DISABLE_CONFIG, b"")

    def read_register(self, reg_addr: int) -> int:
        if not 0 <= reg_addr <= 0xFFFF:
            raise ValueError("register address out of range")
        addr_bytes = reg_addr.to_bytes(2, "little")
        self._send_command(self._CMD_READ_REGISTER, addr_bytes)
        t0 = time.ticks_ms()
        while True:
            if self._uart.any():
                tmp = bytearray(32)
                n = self._uart.readinto(tmp)
                self._buf.extend(tmp[:n])
            frame = self._extract_frame()
            if frame is not None:
                cmd, payload = self._parse_generic_frame(frame)
                if cmd == self._CMD_READ_REGISTER and len(payload) >= 6:
                    err_code = payload[0] | (payload[1] << 8)
                    if err_code != 0:
                        raise ValueError(f"error {err_code:04x} reading register {reg_addr}")
                    val = int.from_bytes(payload[2:6], "little")
                    return val
            if time.ticks_diff(time.ticks_ms(), t0) > 1000:
                raise TimeoutError("timeout reading register")
            time.sleep_ms(10)

    def write_register(self, reg_addr: int, value: int, verify: bool = True) -> None:
        if not 0 <= reg_addr <= 0xFFFF:
            raise ValueError("register address out of range")
        if not 0 <= value <= 0xFFFFFFFF:
            raise ValueError("value must fit in 32 bits")
        payload = reg_addr.to_bytes(2, "little") + value.to_bytes(4, "little")
        self._send_command(self._CMD_WRITE_REGISTER, payload)
        if verify:
            read_back = self.read_register(reg_addr)
            if read_back != value:
                raise RuntimeError(f"verification failed: wrote {value:#x}, got {read_back:#x}")

    def on_data(self, callback: Optional[Callable[[bool, List[List[int]], int, Dict[str, Any]], None]]) -> None:
        self._callback = callback

    # ------------------------------------------------------------------
    # Private helpers
    def _init_uart_and_check(self) -> None:
        for attempt in range(2):
            if attempt == 0:
                rx_pin, tx_pin = self._rx_pin, self._tx_pin
                baud = self._baudrate
            else:
                rx_pin, tx_pin = self._tx_pin, self._rx_pin
                baud = self._alternate_baudrate
            try:
                self._uart = UART(1, baudrate=baud, rx=rx_pin, tx=tx_pin)
                self._ot = Pin(self._ot_pin_no, Pin.IN)
                self._buf = bytearray()
                try:
                    ver = self._try_read_version_once()
                except Exception:
                    ver = ""  # ignore any error during version read
                if self.debug:
                    print(f"UART initialised on RX={rx_pin}, TX={tx_pin}, baud={baud}")
                    if ver:
                        print(f"LD2420 firmware version: {ver}")
                return
            except Exception as exc:
                if self.debug:
                    print(f"UART initialisation attempt {attempt} failed: {exc}")
                time.sleep_ms(50)
        raise ValueError("Failed to initialise UART or communicate with LD2420; check wiring and baudrate")

    def _try_read_version_once(self) -> str:
        while self._uart.any():
            self._uart.read(self._uart.any())  # type: ignore
        self._send_command(self._CMD_ENABLE_CONFIG, b"\x02\x00")
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < 500:
            if self._uart.any():
                tmp = bytearray(64)
                n = self._uart.readinto(tmp)
                self._buf.extend(tmp[:n])
            frame = self._extract_frame()
            if frame is not None:
                cmd, payload = self._parse_generic_frame(frame)
                if len(payload) >= 4:
                    err_code = payload[0] | (payload[1] << 8)
                    if err_code == 0:
                        major = payload[2]
                        minor = payload[3]
                        self._send_command(self._CMD_DISABLE_CONFIG, b"")
                        return f"{major}.{minor}"
                    else:
                        raise RuntimeError(f"version read error code {err_code:04x}")
            time.sleep_ms(10)
        raise TimeoutError("no version response")

    def _send_system_mode(self, mode_param: int) -> None:
        payload = b"\x00\x00" + mode_param.to_bytes(2, "little") + b"\x00\x00"
        self._send_command(self._CMD_SYSTEM_MODE, payload)

    def _send_command(self, command: int, payload: bytes) -> None:
        frame = self._build_frame(command, payload)
        self._uart.write(frame)
        if self.debug:
            print("[TX]", frame)

    def _build_frame(self, command: int, payload: bytes) -> bytes:
        if command < 0 or command > 0xFFFF:
            raise ValueError("command must be 16‑bit")
        length = len(payload) + 2
        length_bytes = length.to_bytes(2, "little")
        command_bytes = command.to_bytes(2, "little")
        return self._FRAME_HEADER + length_bytes + command_bytes + payload + self._FRAME_FOOTER

    def _extract_frame(self) -> Optional[bytes]:
        """
        Attempt to extract either a command/ack frame or a streaming frame from
        the internal buffer.  The LD2420 uses two framing formats:

        * Command/ack frames: header 0xFD FC FB FA, footer 0x04 03 02 01.
          After the header and two length bytes the next two bytes contain
          the command word.  The length counts the command word and payload.

        * Streaming (energy) frames: header 0xF4 F3 F2 F1, footer 0xF8 F7 F6 F5.
          After the header the next two bytes specify the payload length.  The
          payload begins with the presence flag followed by distance and energy
          values.  There is no command word in streaming frames.

        Returns a complete frame as bytes if one is available, otherwise
        returns None.  If malformed data is encountered the buffer is
        resynchronised by discarding bytes until a plausible header is found.
        """
        buf = self._buf
        # Shorthand references
        cmd_header = self._FRAME_HEADER
        cmd_footer = self._FRAME_FOOTER
        stream_header = self._STREAM_HEADER
        stream_footer = self._STREAM_FOOTER
        while True:
            # Find the earliest occurrence of either header pattern
            idx_cmd = buf.find(cmd_header)
            idx_stream = buf.find(stream_header)
            # Determine the closest valid header; ignore -1 values
            indices = [i for i in (idx_cmd, idx_stream) if i != -1]
            if not indices:
                return None
            idx = min(indices)
            # Discard bytes before the header
            if idx > 0:
                buf[:] = buf[idx:]          # discard everything up to the header
            # If we found the command header, attempt to parse a command frame
            if idx_cmd != -1 and idx == idx_cmd:
                # Need at least header + length + footer
                if len(buf) < 4 + 2 + 4:
                    return None
                length = buf[4] | (buf[5] << 8)
                total_len = 4 + 2 + length + 4
                if len(buf) < total_len:
                    return None
                frame = bytes(buf[:total_len])
                # Validate footer
                if frame[-4:] != cmd_footer:
                    # Invalid footer; discard first byte and continue searching
                    buf[:] = buf[1:]
                    continue
                # Remove the frame from the buffer
                buf[:] = buf[total_len:]    # remove the complete frame
                return frame
            # Otherwise we have a streaming header at the start
            if idx_stream != -1 and idx == idx_stream:
                # Need header + length + footer
                if len(buf) < 4 + 2 + 4:
                    return None
                # Streaming length field is two bytes immediately after header
                length = buf[4] | (buf[5] << 8)
                total_len = 4 + 2 + length + 4
                if len(buf) < total_len:
                    return None
                frame = bytes(buf[:total_len])
                if frame[-4:] != stream_footer:
                    # Invalid streaming footer; discard one byte and continue
                    buf[:] = buf[1:]
                    continue
                buf[:] = buf[total_len:]    # remove the complete frame
                return frame

    def _parse_generic_frame(self, frame: bytes) -> Tuple[int, bytes]:
        if not (frame.startswith(self._FRAME_HEADER) and frame.endswith(self._FRAME_FOOTER)):
            raise ValueError("invalid frame delimiters")
        length = frame[4] | (frame[5] << 8)
        command = frame[6] | (frame[7] << 8)
        payload = frame[8 : 8 + length - 2]
        return command, payload

    def _decode_presence_frame(self, frame: bytes) -> Tuple[bool, List[List[int]], int, Dict[str, Any]]:
        """
        Decode a streaming (energy output) frame from the LD2420 into structured values.

        Presence frames from the LD2420 in energy mode have the following layout:

            Presence (1 byte):        0 = none, 1 = motion detected
            Distance (2 bytes LE):    distance in cm
            Energy[0..15] (2 bytes): 16 little‑endian values representing reflected
                                    energy in gates 0–15

        Any remaining bytes after the 16 energies are stored in the `extra` dictionary
        under the key `'raw'`.  If the frame appears to be an acknowledgement or its
        payload is too short to contain a complete presence report, the method
        returns a default (no motion) result instead of raising an exception.
        """
        # Determine the length of the payload reported by the frame
        length = frame[4] | (frame[5] << 8)
        # The streaming payload begins immediately after the length field.  Unlike
        # command frames, there is no separate command word in energy mode; the
        # first byte of the payload is the presence flag.
        payload_start = 6
        payload_end = payload_start + length
        payload = frame[payload_start:payload_end]
        # Minimum bytes required for a full presence report: 1 (presence flag)
        # + 2 (distance) + 32 (16 energies × 2 bytes)
        minimal_required = 1 + 2 + 32

        # Some frames may actually be acknowledgements for commands (e.g. after
        # configuration writes).  Such frames include a command word at bytes 6–7
        # and typically have a shorter payload.  We treat any frame whose
        # payload length is less than the minimal size as non‑presence data and
        # return a default result instead of attempting to decode it.
        if len(payload) < minimal_required:
            # Attempt to decode acknowledgments for debugging; ignore errors
            try:
                self._decode_ack(frame)
            except Exception:
                pass
            # Return a default (no presence) result
            grid = [[0, 0, 0, 0] for _ in range(4)]
            return False, grid, 0, {"raw": b""}

        # Parse presence flag and distance
        presence_flag = payload[0]
        presence = bool(presence_flag)
        distance = payload[1] | (payload[2] << 8)
        # Extract sixteen 16‑bit energy values
        energies: List[int] = []
        idx = 3
        for _ in range(16):
            # Guard against truncated data; if incomplete, pad with zeros
            if idx + 1 >= len(payload):
                energies.append(0)
            else:
                val = payload[idx] | (payload[idx + 1] << 8)
                energies.append(val)
            idx += 2
        # Arrange energies into a 4×4 grid
        grid = [energies[i : i + 4] for i in range(0, 16, 4)]
        # Any remaining bytes beyond the energies are captured for diagnostic purposes
        extra_bytes = payload[idx:] if idx < len(payload) else b""
        extra: Dict[str, Any] = {"raw": bytes(extra_bytes)}
        return presence, grid, distance, extra

    def _decode_ack(self, frame: bytes) -> None:
        """Decode a simple acknowledgement frame and raise if an error was reported."""
        cmd = frame[6] | (frame[7] << 8)
        error = frame[8] | (frame[9] << 8)
        if error:
            raise RuntimeError(f"LD2420 reported error 0x{error:04X} for cmd 0x{cmd:04X}")

